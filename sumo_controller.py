import os
import shutil
from dataclasses import dataclass
from pathlib import Path

import traci


GUI_VIEW_ID = "View #0"
GUI_VIEW_CENTER = (700.0, 0.0)
GUI_VIEW_ZOOM = 350.0
GUI_VEHICLE_COLORS = {
    "emergency_truck": (220, 35, 35, 255),
    "car_01": (0, 150, 255, 255),
    "car_02": (255, 145, 0, 255),
    "car_03": (40, 190, 90, 255),
    "car_04": (180, 70, 220, 255),
    "car_05": (0, 200, 190, 255),
    "car_06": (245, 205, 45, 255),
}


@dataclass(frozen=True)
class VehicleSnapshot:
    vehicle_id: str
    position: float
    speed: float
    road_id: str
    lane_id: str
    length: float
    vehicle_type: str


def find_sumo_binary(gui: bool = False) -> str:
    executable = "sumo-gui" if gui else "sumo"
    if os.name == "nt":
        executable += ".exe"

    sumo_home = os.environ.get("SUMO_HOME")
    if sumo_home:
        candidate = Path(sumo_home) / "bin" / executable
        if candidate.is_file():
            return str(candidate)

    binary = shutil.which(executable)
    if binary:
        return binary

    raise FileNotFoundError(
        f"{executable} was not found. Set SUMO_HOME or add "
        "SUMO_HOME\\bin to PATH."
    )


class SumoController:
    def __init__(
        self,
        config_file: Path,
        gui: bool = False,
        gui_step_delay_ms: int = 300,
        seed: int = 42,
    ):
        if gui_step_delay_ms < 0:
            raise ValueError("gui_step_delay_ms must be non-negative")
        self.config_file = config_file.resolve()
        self.gui = gui
        self.gui_step_delay_ms = gui_step_delay_ms
        self.seed = seed
        self._started = False

    def __enter__(self):
        command = [
            find_sumo_binary(gui=self.gui),
            "-c",
            str(self.config_file),
            "--no-step-log",
            "true",
            "--duration-log.disable",
            "true",
            "--seed",
            str(self.seed),
        ]
        if self.gui and self.gui_step_delay_ms:
            command.extend(["--delay", str(self.gui_step_delay_ms)])

        traci.start(command)
        self._started = True
        if self.gui:
            traci.gui.setOffset(GUI_VIEW_ID, *GUI_VIEW_CENTER)
            traci.gui.setZoom(GUI_VIEW_ID, GUI_VIEW_ZOOM)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self._started:
            traci.close()
            self._started = False

    @property
    def time(self) -> float:
        return float(traci.simulation.getTime())

    @property
    def min_expected_number(self) -> int:
        return int(traci.simulation.getMinExpectedNumber())

    def vehicle_ids(self) -> tuple[str, ...]:
        return tuple(traci.vehicle.getIDList())

    def vehicle_exists(self, vehicle_id: str) -> bool:
        return vehicle_id in traci.vehicle.getIDList()

    def snapshot(self, vehicle_id: str) -> VehicleSnapshot:
        return VehicleSnapshot(
            vehicle_id=vehicle_id,
            position=float(traci.vehicle.getLanePosition(vehicle_id)),
            speed=float(traci.vehicle.getSpeed(vehicle_id)),
            road_id=traci.vehicle.getRoadID(vehicle_id),
            lane_id=traci.vehicle.getLaneID(vehicle_id),
            length=float(traci.vehicle.getLength(vehicle_id)),
            vehicle_type=traci.vehicle.getTypeID(vehicle_id),
        )

    def lock_lane(self, vehicle_id: str) -> None:
        traci.vehicle.setLaneChangeMode(vehicle_id, 0)

    def track_vehicle(self, vehicle_id: str) -> None:
        if self.gui:
            traci.gui.trackVehicle(GUI_VIEW_ID, vehicle_id)

    def set_presentation_color(self, vehicle_id: str) -> None:
        if self.gui and vehicle_id in GUI_VEHICLE_COLORS:
            traci.vehicle.setColor(vehicle_id, GUI_VEHICLE_COLORS[vehicle_id])

    def brake(self, vehicle_id: str, duration: float) -> None:
        traci.vehicle.slowDown(vehicle_id, 0.0, duration)

    def hold_brake(self, vehicle_id: str) -> None:
        traci.vehicle.setSpeed(vehicle_id, 0.0)

    def step(self) -> None:
        traci.simulationStep()

    def collision_pairs(self) -> tuple[tuple[str, str], ...]:
        pairs = set()
        for collision in traci.simulation.getCollisions():
            if collision.collider and collision.victim:
                pairs.add(tuple(sorted((collision.collider, collision.victim))))
        return tuple(sorted(pairs))
