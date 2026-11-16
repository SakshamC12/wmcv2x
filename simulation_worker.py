from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
import math
import queue
import threading
import time
import traceback
from typing import Callable

from sumo_simulation import (
    SimulationSettings,
    run_sumo_comparison,
    run_sumo_simulation,
)
from v2x_communication import CommunicationMode


class WorkerState(str, Enum):
    READY = "ready"
    RUNNING = "running"
    PAUSE_REQUESTED = "pause_requested"
    PAUSED = "paused"
    RESUME_REQUESTED = "resume_requested"
    STOPPING = "stopping"
    STOPPED = "stopped"
    COMPLETED = "completed"
    FAILED = "failed"


class WorkerCommand(str, Enum):
    PAUSE = "pause"
    RESUME = "resume"
    STOP = "stop"


@dataclass(frozen=True)
class WorkerEvent:
    event_type: str
    simulation_time: float | None
    data: dict[str, object]
    run_id: int


@dataclass(frozen=True)
class VehicleTelemetry:
    vehicle_id: str
    position: float
    speed: float
    road_id: str
    lane_id: str
    length: float
    vehicle_type: str


@dataclass(frozen=True)
class TelemetrySnapshot:
    simulation_time: float
    vehicles: tuple[VehicleTelemetry, ...]
    communication_mode: str
    rsu: dict[str, object]
    run_id: int


SimulationRunner = Callable[..., dict[str, object]]

_ACTIVE_RUN_LOCK = threading.Lock()


class SimulationWorker:
    """Runs one headless SUMO simulation, with all TraCI access on its thread."""

    def __init__(
        self,
        settings: SimulationSettings | None = None,
        *,
        simulation_runner: SimulationRunner = run_sumo_simulation,
        comparison_runner: SimulationRunner = run_sumo_comparison,
        communication_mode: CommunicationMode | str = CommunicationMode.V2V,
        comparison_modes: tuple[CommunicationMode | str, ...] | None = None,
        telemetry_step_delay_seconds: float = 0.0,
    ):
        if (
            not math.isfinite(telemetry_step_delay_seconds)
            or telemetry_step_delay_seconds < 0
        ):
            raise ValueError(
                "telemetry_step_delay_seconds must be finite and non-negative"
            )
        self.settings = settings or SimulationSettings()
        self._simulation_runner = simulation_runner
        self._comparison_runner = comparison_runner
        self.communication_mode = CommunicationMode(communication_mode)
        self.comparison_modes = comparison_modes
        self.telemetry_step_delay_seconds = telemetry_step_delay_seconds
        self.command_queue: queue.Queue[WorkerCommand] = queue.Queue()
        self.telemetry_queue: queue.Queue[TelemetrySnapshot] = queue.Queue(
            maxsize=1
        )
        self.event_queue: queue.Queue[WorkerEvent] = queue.Queue()
        self._state = WorkerState.READY
        self._state_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._results: dict[str, object] | None = None
        self._error: str | None = None
        self._run_id = 0
        self._stop_requested = threading.Event()

    @property
    def run_id(self) -> int:
        with self._state_lock:
            return self._run_id

    @property
    def state(self) -> WorkerState:
        with self._state_lock:
            return self._state

    @property
    def results(self) -> dict[str, object] | None:
        with self._state_lock:
            return self._results

    @property
    def error(self) -> str | None:
        with self._state_lock:
            return self._error

    def start(self) -> None:
        global _ACTIVE_RUN_LOCK

        with self._state_lock:
            if self._state is not WorkerState.READY:
                raise RuntimeError(
                    f"Cannot start a worker in state {self._state.value}; reset it first"
                )
            if not _ACTIVE_RUN_LOCK.acquire(blocking=False):
                raise RuntimeError("Another SUMO simulation is already active")

            self._run_id += 1
            self._state = WorkerState.RUNNING
            self._publish_state(WorkerState.RUNNING)
            self._thread = threading.Thread(
                target=self._run,
                name="sumo-simulation-worker",
                daemon=True,
            )
            try:
                self._thread.start()
            except BaseException:
                self._state = WorkerState.READY
                _ACTIVE_RUN_LOCK.release()
                raise

    def pause(self) -> None:
        with self._state_lock:
            if self._state is not WorkerState.RUNNING:
                raise RuntimeError(
                    f"Cannot pause a worker in state {self._state.value}"
                )
            self._state = WorkerState.PAUSE_REQUESTED
            self.command_queue.put(WorkerCommand.PAUSE)
            self._publish_state(WorkerState.PAUSE_REQUESTED)

    def resume(self) -> None:
        with self._state_lock:
            if self._state not in (
                WorkerState.PAUSE_REQUESTED,
                WorkerState.PAUSED,
            ):
                raise RuntimeError(
                    f"Cannot resume a worker in state {self._state.value}"
                )
            self._state = WorkerState.RESUME_REQUESTED
            self.command_queue.put(WorkerCommand.RESUME)
            self._publish_state(WorkerState.RESUME_REQUESTED)

    def stop(self) -> None:
        with self._state_lock:
            if self._state in (
                WorkerState.STOPPED,
                WorkerState.COMPLETED,
                WorkerState.FAILED,
                WorkerState.READY,
            ):
                return
            if self._state is not WorkerState.STOPPING:
                self._state = WorkerState.STOPPING
                self._stop_requested.set()
                self.command_queue.put(WorkerCommand.STOP)
                self._publish_state(WorkerState.STOPPING)

    def join(self, timeout: float | None = None) -> bool:
        thread = self._thread
        if thread is None:
            return True
        thread.join(timeout)
        return not thread.is_alive()

    def reset(self, timeout: float | None = 10.0) -> None:
        if self._thread is not None and self._thread.is_alive():
            self.stop()
            if not self.join(timeout):
                raise TimeoutError("SUMO worker did not stop before reset timed out")

        with self._state_lock:
            self._thread = None
            self._results = None
            self._error = None
            self._stop_requested.clear()
            self._state = WorkerState.READY
            self._drain_queue(self.command_queue)
            self._drain_queue(self.telemetry_queue)
            self._drain_queue(self.event_queue)

    @staticmethod
    def _drain_queue(target: queue.Queue) -> None:
        while True:
            try:
                target.get_nowait()
            except queue.Empty:
                return

    def _publish_state(self, state: WorkerState) -> None:
        self.event_queue.put(
            WorkerEvent(
                event_type="WORKER_STATE",
                simulation_time=None,
                data={"state": state.value},
                run_id=self._run_id,
            )
        )

    def _publish_event(self, event: dict[str, object]) -> None:
        event_copy = deepcopy(event)
        event_type = str(event_copy.pop("event_type"))
        raw_time = event_copy.pop("time", None)
        simulation_time = (
            float(raw_time) if raw_time is not None else None
        )
        self.event_queue.put(
            WorkerEvent(
                event_type=event_type,
                simulation_time=simulation_time,
                data=event_copy,
                run_id=self._run_id,
            )
        )

    def _publish_telemetry(self, snapshot: dict[str, object]) -> None:
        vehicles = tuple(
            VehicleTelemetry(**vehicle)
            for vehicle in snapshot["vehicles"]
        )
        telemetry = TelemetrySnapshot(
            simulation_time=float(snapshot["time"]),
            vehicles=vehicles,
            communication_mode=str(
                snapshot.get("communication_mode", self.communication_mode.value)
            ),
            rsu=dict(snapshot.get("rsu", {})),
            run_id=self.run_id,
        )

        try:
            self.telemetry_queue.put_nowait(telemetry)
        except queue.Full:
            try:
                self.telemetry_queue.get_nowait()
            except queue.Empty:
                pass
            self.telemetry_queue.put_nowait(telemetry)
        if self.telemetry_step_delay_seconds:
            time.sleep(self.telemetry_step_delay_seconds)

    def _set_state(self, state: WorkerState) -> None:
        with self._state_lock:
            self._state = state
            self._publish_state(state)

    def _handle_command(self, command: WorkerCommand) -> bool:
        if command is WorkerCommand.PAUSE:
            self._set_state(WorkerState.PAUSED)
            return True
        if command is WorkerCommand.RESUME:
            self._set_state(WorkerState.RUNNING)
            return True
        if command is WorkerCommand.STOP:
            return False
        return True

    def _before_step(self) -> bool:
        if self._stop_requested.is_set():
            return False

        while True:
            try:
                command = self.command_queue.get_nowait()
            except queue.Empty:
                return not self._stop_requested.is_set()

            if command is WorkerCommand.PAUSE:
                if not self._handle_command(command):
                    return False
                while True:
                    command = self.command_queue.get()
                    if not self._handle_command(command):
                        return False
                    if command is WorkerCommand.RESUME:
                        break
            elif not self._handle_command(command):
                return False

    def _run(self) -> None:
        final_state = WorkerState.FAILED
        try:
            runner = (
                self._comparison_runner
                if self.comparison_modes is not None
                else self._simulation_runner
            )
            runner_options = {
                "on_step": self._publish_telemetry,
                "on_event": self._publish_event,
                "should_continue": self._before_step,
            }
            if self.comparison_modes is not None:
                runner_options["modes"] = self.comparison_modes
            else:
                runner_options["gui"] = False
                runner_options["communication_mode"] = self.communication_mode
            results = runner(self.settings, **runner_options)
            with self._state_lock:
                self._results = results
            if results.get("stopped") or self._stop_requested.is_set():
                final_state = WorkerState.STOPPED
            else:
                final_state = WorkerState.COMPLETED
        except Exception as error:
            with self._state_lock:
                self._error = "".join(
                    traceback.format_exception(type(error), error, error.__traceback__)
                )
            self._publish_event(
                {
                    "event_type": "SIMULATION_ERROR",
                    "error": str(error),
                    "traceback": self._error,
                }
            )
        finally:
            self._set_state(final_state)
            _ACTIVE_RUN_LOCK.release()
