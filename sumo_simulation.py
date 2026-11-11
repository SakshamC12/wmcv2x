import json
from dataclasses import dataclass
from pathlib import Path

from metrics import calculate_sumo_metrics
from sumo_controller import SumoController
from v2x_communication import ControlledV2XChannel
from v2x_logic import assess_collision_risk
from v2x_message import create_safety_message


SCENARIO_DIR = Path(__file__).parent / "sumo_scenario"
EXPECTED_VEHICLE_IDS = {
    "emergency_truck",
    "car_01",
    "car_02",
    "car_03",
    "car_04",
    "car_05",
    "car_06",
}


@dataclass(frozen=True)
class SimulationSettings:
    config_file: Path = SCENARIO_DIR / "highway.sumocfg"
    end_time: float = 20.0
    emergency_brake_time: float = 5.0
    emergency_brake_duration: float = 4.0
    reaction_delay: float = 0.5
    response_brake_duration: float = 3.0
    communication_range_m: float = 250.0
    maximum_ttc_seconds: float = 15.0

    def __post_init__(self):
        if self.end_time <= self.emergency_brake_time + self.reaction_delay:
            raise ValueError("end_time must allow the emergency event and response")
        if self.emergency_brake_time < 0 or self.reaction_delay < 0:
            raise ValueError("event time and reaction delay must be non-negative")
        if self.emergency_brake_duration <= 0 or self.response_brake_duration <= 0:
            raise ValueError("braking durations must be positive")
        if self.maximum_ttc_seconds <= 0:
            raise ValueError("maximum_ttc_seconds must be positive")


def run_sumo_simulation(
    settings: SimulationSettings | None = None,
) -> dict[str, object]:
    settings = settings or SimulationSettings()
    channel = ControlledV2XChannel(settings.communication_range_m)
    history = []
    events = []
    message_log = []
    warning_records = []
    response_vehicle_ids = set()
    observed_vehicle_ids = set()
    collision_pairs = set()
    pending_responses: dict[str, float] = {}
    lane_locked_ids = set()
    emergency_event_sent = False

    with SumoController(settings.config_file) as controller:
        while controller.time < settings.end_time:
            simulation_time = controller.time
            vehicle_ids = controller.vehicle_ids()
            snapshots = {
                vehicle_id: controller.snapshot(vehicle_id)
                for vehicle_id in vehicle_ids
            }
            observed_vehicle_ids.update(vehicle_ids)

            for vehicle_id in vehicle_ids:
                if vehicle_id not in lane_locked_ids:
                    controller.lock_lane(vehicle_id)
                    lane_locked_ids.add(vehicle_id)

            if (
                not emergency_event_sent
                and simulation_time >= settings.emergency_brake_time
            ):
                truck = snapshots.get("emergency_truck")
                if truck is None:
                    raise RuntimeError(
                        "The emergency truck was not present when braking was scheduled"
                    )

                controller.brake(
                    truck.vehicle_id,
                    settings.emergency_brake_duration,
                )
                outgoing_message = create_safety_message(truck, simulation_time)
                outgoing_message.update(
                    {
                        "road_id": truck.road_id,
                        "lane_id": truck.lane_id,
                        "vehicle_length": truck.length,
                        "emergency_braking_duration_s": (
                            settings.emergency_brake_duration
                        ),
                    }
                )
                events.append(
                    {
                        "event_type": "EMERGENCY_BRAKING",
                        "vehicle_id": truck.vehicle_id,
                        "time": simulation_time,
                        "message_id": outgoing_message["message_id"],
                    }
                )

                deliveries = channel.broadcast(
                    outgoing_message,
                    tuple(snapshots.values()),
                    simulation_time,
                )
                for delivery in deliveries:
                    receiver = snapshots[delivery.recipient_id]
                    risk = assess_collision_risk(
                        delivery.message,
                        receiver,
                        settings.maximum_ttc_seconds,
                    )
                    delivery.message["same_lane_risk"] = risk.same_lane
                    delivery.message["ttc_at_warning_s"] = risk.ttc_seconds
                    delivery.message["response_recommended"] = risk.should_brake
                    delivery.message["risk_reason"] = risk.reason
                    message_log.append(delivery.message)
                    warning_records.append(
                        {
                            "vehicle_id": delivery.recipient_id,
                            "warning_distance_m": delivery.message[
                                "warning_distance_m"
                            ],
                            "ttc_at_warning_s": risk.ttc_seconds,
                            "response_recommended": risk.should_brake,
                        }
                    )
                    if risk.should_brake:
                        pending_responses[delivery.recipient_id] = (
                            simulation_time + settings.reaction_delay
                        )

                emergency_event_sent = True

            for vehicle_id, response_time in tuple(pending_responses.items()):
                if simulation_time < response_time:
                    continue
                if controller.vehicle_exists(vehicle_id):
                    controller.brake(
                        vehicle_id,
                        settings.response_brake_duration,
                    )
                    response_vehicle_ids.add(vehicle_id)
                    events.append(
                        {
                            "event_type": "V2X_BRAKE_RESPONSE",
                            "vehicle_id": vehicle_id,
                            "time": simulation_time,
                        }
                    )
                del pending_responses[vehicle_id]

            controller.step()
            collision_pairs.update(controller.collision_pairs())
            frame_vehicles = [
                controller.snapshot(vehicle_id).__dict__
                for vehicle_id in controller.vehicle_ids()
            ]
            observed_vehicle_ids.update(
                vehicle["vehicle_id"] for vehicle in frame_vehicles
            )
            history.append(
                {
                    "time": controller.time,
                    "vehicles": frame_vehicles,
                }
            )

            if controller.min_expected_number == 0:
                break

    missing_vehicles = EXPECTED_VEHICLE_IDS - observed_vehicle_ids
    if missing_vehicles:
        raise RuntimeError(
            f"SUMO scenario did not load expected vehicles: {sorted(missing_vehicles)}"
        )
    if not emergency_event_sent:
        raise RuntimeError("The scheduled emergency-braking event did not occur")

    metrics = calculate_sumo_metrics(
        messages_transmitted=channel.messages_transmitted,
        transmission_attempts=channel.transmission_attempts,
        messages_received=channel.messages_received,
        latency_samples_ms=channel.latency_samples_ms,
        warning_records=warning_records,
        response_vehicle_ids=response_vehicle_ids,
        collision_pairs=collision_pairs,
    )
    return {
        "history": history,
        "events": events,
        "message_log": message_log,
        "warning_records": warning_records,
        "vehicle_ids": sorted(observed_vehicle_ids),
        "responded_vehicle_ids": sorted(response_vehicle_ids),
        "collision_pairs": sorted(collision_pairs),
        "metrics": metrics,
    }


def main() -> None:
    results = run_sumo_simulation()
    print(
        json.dumps(
            {
                "vehicle_ids": results["vehicle_ids"],
                "responded_vehicle_ids": results["responded_vehicle_ids"],
                "events": results["events"],
                "metrics": results["metrics"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
