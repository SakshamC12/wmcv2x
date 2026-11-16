import threading
import time
import unittest
from queue import Empty
from pathlib import Path

import traci

from simulation_worker import (
    SimulationWorker,
    TelemetrySnapshot,
    WorkerState,
)
from sumo_simulation import SimulationSettings, run_sumo_simulation
from v2x_communication import CommunicationMode


def _fake_runner(
    settings,
    *,
    gui,
    communication_mode,
    on_step,
    on_event,
    should_continue,
):
    del settings, gui
    step = 0
    while should_continue():
        step += 1
        on_step(
            {
                "time": step / 10,
                "vehicles": [
                    {
                        "vehicle_id": "test_car",
                        "position": float(step),
                        "speed": 10.0,
                        "road_id": "test_road",
                        "lane_id": "test_road_0",
                        "length": 5.0,
                        "vehicle_type": "passenger",
                    }
                ],
                "communication_mode": communication_mode.value,
                "rsu": {},
            }
        )
        on_event(
            {
                "event_type": "TEST_EVENT",
                "time": step / 10,
                "step": step,
            }
        )
        time.sleep(0.01)
    return {"stopped": True}


def _wait_for_state(worker, state, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if worker.state is state:
            return
        time.sleep(0.005)
    raise AssertionError(f"Worker did not reach state {state.value}")


def _latest_telemetry(worker):
    latest = None
    while True:
        try:
            latest = worker.telemetry_queue.get_nowait()
        except Empty:
            return latest


class SimulationWorkerLifecycleTest(unittest.TestCase):
    def test_pause_resume_stop_and_reset(self):
        worker = SimulationWorker(simulation_runner=_fake_runner)
        worker.start()
        try:
            initial = worker.telemetry_queue.get(timeout=5)

            worker.pause()
            _wait_for_state(worker, WorkerState.PAUSED)
            paused = _latest_telemetry(worker) or initial
            time.sleep(0.05)
            self.assertIsNone(_latest_telemetry(worker))

            worker.resume()
            deadline = time.monotonic() + 5
            advanced = False
            while time.monotonic() < deadline:
                telemetry = _latest_telemetry(worker)
                if telemetry and telemetry.simulation_time > paused.simulation_time:
                    advanced = True
                    break
                time.sleep(0.005)
            self.assertTrue(advanced)
            self.assertLessEqual(worker.telemetry_queue.maxsize, 1)

            worker.reset(timeout=5)
            self.assertEqual(worker.state, WorkerState.READY)
            self.assertTrue(worker.telemetry_queue.empty())
            self.assertTrue(worker.event_queue.empty())

            worker.start()
            worker.stop()
            self.assertTrue(worker.join(timeout=5))
            self.assertEqual(worker.state, WorkerState.STOPPED)
            self.assertEqual(initial.vehicles[0].lane_id, "test_road_0")
        finally:
            if worker.state in (
                WorkerState.RUNNING,
                WorkerState.PAUSE_REQUESTED,
                WorkerState.PAUSED,
                WorkerState.RESUME_REQUESTED,
                WorkerState.STOPPING,
            ):
                worker.stop()
                worker.join(timeout=5)

    def test_only_one_sumo_run_can_be_active(self):
        entered = threading.Event()
        release = threading.Event()

        def blocked_runner(
            settings,
            *,
            gui,
            communication_mode,
            on_step,
            on_event,
            should_continue,
        ):
            del settings, gui, communication_mode, on_step, on_event
            entered.set()
            release.wait(timeout=5)
            return {"stopped": not should_continue()}

        first = SimulationWorker(simulation_runner=blocked_runner)
        second = SimulationWorker(simulation_runner=_fake_runner)
        first.start()
        self.assertTrue(entered.wait(timeout=5))
        try:
            with self.assertRaisesRegex(RuntimeError, "already active"):
                second.start()
        finally:
            release.set()
            self.assertTrue(first.join(timeout=5))

        second.start()
        second.stop()
        self.assertTrue(second.join(timeout=5))

    def test_real_sumo_publishes_telemetry_and_retains_events(self):
        worker = SimulationWorker(
            SimulationSettings(end_time=10.0),
        )
        worker.start()
        self.assertTrue(worker.join(timeout=30))

        self.assertEqual(worker.state, WorkerState.COMPLETED)
        telemetry = _latest_telemetry(worker)
        self.assertIsInstance(telemetry, TelemetrySnapshot)
        self.assertEqual(telemetry.simulation_time, 10.0)
        self.assertEqual(len(telemetry.vehicles), 7)
        self.assertTrue(all(vehicle.lane_id for vehicle in telemetry.vehicles))
        self.assertTrue(all(vehicle.road_id for vehicle in telemetry.vehicles))

        events = []
        while not worker.event_queue.empty():
            events.append(worker.event_queue.get_nowait())
        emergency_events = [
            event for event in events
            if event.event_type == "EMERGENCY_BRAKING"
        ]
        response_events = [
            event for event in events
            if event.event_type == "V2X_BRAKE_RESPONSE"
        ]
        received_events = [
            event for event in events
            if event.event_type == "V2X_MESSAGE_RECEIVED"
        ]
        self.assertEqual([event.simulation_time for event in emergency_events], [5.0])
        self.assertEqual(
            [event.simulation_time for event in response_events],
            [5.5, 5.5, 5.5],
        )
        self.assertEqual(len(received_events), 6)

        worker.reset()
        self.assertEqual(worker.state, WorkerState.READY)
        worker.start()
        self.assertTrue(worker.join(timeout=30))
        self.assertEqual(worker.state, WorkerState.COMPLETED)
        self.assertFalse(traci.connection.has("default"))

    def test_comparison_runner_receives_only_supported_options(self):
        received_modes = []

        def comparison_runner(
            settings,
            *,
            modes,
            on_step,
            on_event,
            should_continue,
        ):
            del settings, on_step, on_event, should_continue
            received_modes.extend(modes)
            return {"stopped": False, "comparison_results": {}}

        modes = tuple(CommunicationMode)
        worker = SimulationWorker(
            comparison_runner=comparison_runner,
            comparison_modes=modes,
        )
        worker.start()
        self.assertTrue(worker.join(timeout=5))

        self.assertEqual(worker.state, WorkerState.COMPLETED)
        self.assertEqual(received_modes, list(modes))

    def test_real_four_mode_comparison_completes_through_worker(self):
        modes = tuple(CommunicationMode)
        worker = SimulationWorker(
            SimulationSettings(end_time=10.0),
            comparison_modes=modes,
        )
        worker.start()
        self.assertTrue(worker.join(timeout=60))

        self.assertEqual(worker.state, WorkerState.COMPLETED, worker.error)
        results = worker.results["comparison_results"]
        self.assertEqual(
            set(results),
            {mode.value for mode in modes},
        )
        for mode in modes:
            with self.subTest(mode=mode.value):
                result = results[mode.value]
                self.assertEqual(
                    result["communication_mode"],
                    mode.value,
                )
                self.assertIn("metrics", result)
                self.assertIn("message_log", result)

    def test_error_closes_traci_connection(self):
        def raising_callback(snapshot):
            raise RuntimeError("telemetry publication failed")

        def raising_runner(settings, **kwargs):
            kwargs["on_step"] = raising_callback
            return run_sumo_simulation(settings, **kwargs)

        worker = SimulationWorker(
            SimulationSettings(
                config_file=Path(__file__).parent / "sumo_scenario" / "smoke.sumocfg",
                end_time=10.0,
            ),
            simulation_runner=raising_runner,
        )
        worker.start()
        self.assertTrue(worker.join(timeout=30))

        self.assertEqual(worker.state, WorkerState.FAILED)
        self.assertIn("telemetry publication failed", worker.error)
        self.assertFalse(traci.connection.has("default"))
        self.assertTrue(
            any(
                event.event_type == "SIMULATION_ERROR"
                for event in list(worker.event_queue.queue)
            )
        )

    def test_three_real_runs_complete_stop_reset_without_stale_queues(self):
        worker = SimulationWorker(
            SimulationSettings(end_time=10.0),
            telemetry_step_delay_seconds=0.005,
        )
        try:
            worker.start()
            self.assertTrue(worker.join(timeout=15))
            self.assertEqual(worker.state, WorkerState.COMPLETED)
            first_run_id = worker.run_id
            self.assertEqual(first_run_id, 1)
            self.assertEqual(
                _latest_telemetry(worker).run_id,
                first_run_id,
            )
            self.assertTrue(
                all(
                    event.run_id == first_run_id
                    for event in list(worker.event_queue.queue)
                )
            )

            worker.reset()
            self.assertTrue(worker.telemetry_queue.empty())
            self.assertTrue(worker.event_queue.empty())
            self.assertIsNone(worker.results)

            worker.start()
            second_telemetry = worker.telemetry_queue.get(timeout=5)
            self.assertEqual(second_telemetry.run_id, 2)
            worker.stop()
            self.assertTrue(worker.join(timeout=15))
            self.assertEqual(worker.state, WorkerState.STOPPED)
            self.assertFalse(traci.connection.has("default"))
            worker.reset()
            self.assertTrue(worker.telemetry_queue.empty())
            self.assertTrue(worker.event_queue.empty())

            worker.start()
            self.assertTrue(worker.join(timeout=15))
            self.assertEqual(worker.state, WorkerState.COMPLETED)
            self.assertEqual(worker.run_id, 3)
            final_telemetry = _latest_telemetry(worker)
            self.assertEqual(final_telemetry.run_id, 3)
            final_events = list(worker.event_queue.queue)
            self.assertTrue(final_events)
            self.assertTrue(all(event.run_id == 3 for event in final_events))
            self.assertEqual(
                [
                    event.simulation_time
                    for event in final_events
                    if event.event_type == "EMERGENCY_BRAKING"
                ],
                [5.0],
            )
            self.assertFalse(traci.connection.has("default"))
        finally:
            if worker.state in (
                WorkerState.RUNNING,
                WorkerState.PAUSE_REQUESTED,
                WorkerState.PAUSED,
                WorkerState.RESUME_REQUESTED,
                WorkerState.STOPPING,
            ):
                worker.stop()
                worker.join(timeout=15)
            if worker.state is not WorkerState.READY:
                worker.reset(timeout=15)


if __name__ == "__main__":
    unittest.main()
