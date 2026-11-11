import os
import shutil
from dataclasses import dataclass
from pathlib import Path

import traci


@dataclass(frozen=True)
class VehicleSnapshot:
    vehicle_id: str
    position: float
    speed: float
    road_id: str
    lane_id: str
    length: float


def find_sumo_binary() -> str:
    executable = "sumo.exe" if os.name == "nt" else "sumo"
    sumo_home = os.environ.get("SUMO_HOME")
    if sumo_home:
        candidate = Path(sumo_home) / "bin" / executable
        if candidate.is_file():
            return str(candidate)

    binary = shutil.which("sumo")
    if binary:
        return binary

    raise FileNotFoundError(
        "SUMO was not found. Set SUMO_HOME or add SUMO_HOME\\bin to PATH."
    )


class SumoController:
    def __init__(self, config_file: Path):
        self.config_file = config_file.resolve()
        self._started = False

    def __enter__(self):
        traci.start(
            [
                find_sumo_binary(),
                "-c",
                str(self.config_file),
                "--no-step-log",
                "true",
                "--duration-log.disable",
                "true",
            ]
        )
        self._started = True
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
        )

    def lock_lane(self, vehicle_id: str) -> None:
        traci.vehicle.setLaneChangeMode(vehicle_id, 0)

    def brake(self, vehicle_id: str, duration: float) -> None:
        traci.vehicle.slowDown(vehicle_id, 0.0, duration)

    def step(self) -> None:
        traci.simulationStep()

    def collision_pairs(self) -> tuple[tuple[str, str], ...]:
        pairs = set()
        for collision in traci.simulation.getCollisions():
            if collision.collider and collision.victim:
                pairs.add(tuple(sorted((collision.collider, collision.victim))))
        return tuple(sorted(pairs))
