import argparse
from copy import deepcopy
import json
import math
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
import time

from metrics import calculate_sumo_metrics
from sumo_controller import SumoController
from v2x_communication import (
    CommunicationMode,
    ControlledV2XChannel,
    RoadsideUnit,
)
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

StepCallback = Callable[[dict[str, object]], None]
EventCallback = Callable[[dict[str, object]], None]
ContinueCallback = Callable[[], bool]


@dataclass(frozen=True)
class SimulationSettings:
    config_file: Path = SCENARIO_DIR / "highway.sumocfg"
    end_time: float = 20.0
    emergency_brake_time: float = 5.0
    emergency_brake_duration: float = 4.0
    reaction_delay: float = 0.5
    response_brake_duration: float = 3.0
    communication_range_m: float = 250.0
    random_seed: int = 42
    roadside_unit: RoadsideUnit = RoadsideUnit()
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
    *,
    gui: bool = False,
    gui_end_delay: float = 45.0,
    gui_step_delay_ms: int = 300,
    communication_mode: CommunicationMode | str = CommunicationMode.V2V,
    on_step: StepCallback | None = None,
    on_event: EventCallback | None = None,
    should_continue: ContinueCallback | None = None,
) -> dict[str, object]:
    if not math.isfinite(gui_end_delay) or gui_end_delay < 0:
        raise ValueError("gui_end_delay must be a finite non-negative number")
    if gui_step_delay_ms < 0:
        raise ValueError("gui_step_delay_ms must be non-negative")

    settings = settings or SimulationSettings()
    communication_mode = CommunicationMode(communication_mode)
    channel = ControlledV2XChannel(settings.communication_range_m)
    history = []
    events = []
    message_log = []
    warning_records = []
    response_vehicle_ids = set()
    observed_vehicle_ids = set()
    collision_pairs = set()
    pending_responses: dict[str, float] = {}
    pending_brake_holds: dict[str, float] = {}
    lane_locked_ids = set()
    presentation_colored_ids = set()
    camera_tracking_started = False
    emergency_event_sent = False
    stopped = False
    initial_conditions = None
    minimum_separation_m = None
    minimum_ttc_s = None

    with SumoController(
        settings.config_file,
        gui=gui,
        gui_step_delay_ms=gui_step_delay_ms,
        seed=settings.random_seed,
    ) as controller:
        while controller.time < settings.end_time:
            if should_continue is not None and not should_continue():
                stopped = True
                break

            simulation_time = controller.time
            for vehicle_id, hold_time in tuple(pending_brake_holds.items()):
                if simulation_time < hold_time:
                    continue
                if controller.vehicle_exists(vehicle_id):
                    controller.hold_brake(vehicle_id)
                del pending_brake_holds[vehicle_id]

            vehicle_ids = controller.vehicle_ids()
            snapshots = {
                vehicle_id: controller.snapshot(vehicle_id)
                for vehicle_id in vehicle_ids
            }
            observed_vehicle_ids.update(vehicle_ids)

            if gui:
                for vehicle_id in vehicle_ids:
                    if vehicle_id not in presentation_colored_ids:
                        controller.set_presentation_color(vehicle_id)
                        presentation_colored_ids.add(vehicle_id)
                if (
                    not camera_tracking_started
                    and "emergency_truck" in snapshots
                ):
                    controller.track_vehicle("emergency_truck")
                    camera_tracking_started = True

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
                pending_brake_holds[truck.vehicle_id] = (
                    simulation_time + settings.emergency_brake_duration
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
                emergency_event = {
                    "event_type": "EMERGENCY_BRAKING",
                    "vehicle_id": truck.vehicle_id,
                    "time": simulation_time,
                    "message_id": outgoing_message["message_id"],
                }
                events.append(emergency_event)
                if on_event is not None:
                    on_event(emergency_event)

                def process_deliveries(deliveries):
                    for delivery in deliveries:
                        process_delivery(delivery)

                def process_delivery(delivery):
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
                    if on_event is not None:
                        on_event(
                            {
                                "event_type": "V2X_MESSAGE_RECEIVED",
                                "time": simulation_time,
                                "vehicle_id": delivery.recipient_id,
                                "communication_type": delivery.message.get(
                                    "communication_type", "V2V"
                                ),
                                "message": delivery.message,
                            }
                        )
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

                if communication_mode.enables_v2v:
                    v2v_deliveries = channel.broadcast(
                        outgoing_message,
                        tuple(snapshots.values()),
                        simulation_time,
                        communication_type="V2V",
                    )
                    if on_event is not None:
                        for delivery in v2v_deliveries:
                            on_event(
                                {
                                    "event_type": "V2X_TRANSMISSION",
                                    "time": simulation_time,
                                    "communication_type": "V2V",
                                    "sender_id": truck.vehicle_id,
                                    "receiver_id": delivery.recipient_id,
                                    "sender_position": truck.position,
                                    "receiver_position": snapshots[
                                        delivery.recipient_id
                                    ].position,
                                    "message_id": outgoing_message["message_id"],
                                }
                            )
                    process_deliveries(v2v_deliveries)

                if communication_mode.enables_v2i:
                    rsu = settings.roadside_unit
                    rsu_distance = abs(truck.position - rsu.position)
                    if (
                        truck.road_id == rsu.road_id
                        and rsu_distance <= rsu.communication_range_m
                    ):
                        v2i_delivery = channel.unicast(
                            outgoing_message,
                            rsu.unit_id,
                            simulation_time,
                            communication_type="V2I",
                            warning_distance_m=rsu_distance,
                        )
                        if on_event is not None:
                            on_event(
                                {
                                    "event_type": "V2X_TRANSMISSION",
                                    "time": simulation_time,
                                    "communication_type": "V2I",
                                    "sender_id": truck.vehicle_id,
                                    "receiver_id": rsu.unit_id,
                                    "sender_position": truck.position,
                                    "receiver_position": rsu.position,
                                    "message_id": outgoing_message["message_id"],
                                }
                            )
                            on_event(
                                {
                                    "event_type": "RSU_MESSAGE_RECEIVED",
                                    "time": simulation_time,
                                    "sender_id": truck.vehicle_id,
                                    "receiver_id": rsu.unit_id,
                                    "communication_type": "V2I",
                                    "message": v2i_delivery.message,
                                }
                            )

                        if on_event is not None:
                            on_event(
                                {
                                    "event_type": "RSU_PROCESSED_WARNING",
                                    "time": simulation_time,
                                    "sender_id": rsu.unit_id,
                                    "receiver_id": rsu.unit_id,
                                    "communication_type": "I2V",
                                    "message_id": outgoing_message["message_id"],
                                }
                            )

                        relayed_message = deepcopy(outgoing_message)
                        relayed_message["sender_id"] = rsu.unit_id
                        relayed_message["source_vehicle_id"] = truck.vehicle_id
                        i2v_deliveries = channel.broadcast(
                            relayed_message,
                            tuple(snapshots.values()),
                            simulation_time,
                            communication_type="I2V",
                            transmitter_position=rsu.position,
                        )
                        if on_event is not None:
                            for delivery in i2v_deliveries:
                                on_event(
                                    {
                                        "event_type": "V2X_TRANSMISSION",
                                        "time": simulation_time,
                                        "communication_type": "I2V",
                                        "sender_id": rsu.unit_id,
                                        "receiver_id": delivery.recipient_id,
                                        "sender_position": rsu.position,
                                        "receiver_position": snapshots[
                                            delivery.recipient_id
                                        ].position,
                                        "message_id": outgoing_message["message_id"],
                                    }
                                )
                        process_deliveries(i2v_deliveries)

                emergency_event_sent = True

            for vehicle_id, response_time in tuple(pending_responses.items()):
                if simulation_time < response_time:
                    continue
                if controller.vehicle_exists(vehicle_id):
                    controller.brake(
                        vehicle_id,
                        settings.response_brake_duration,
                    )
                    pending_brake_holds[vehicle_id] = (
                        simulation_time + settings.response_brake_duration
                    )
                    response_vehicle_ids.add(vehicle_id)
                    response_event = {
                        "event_type": "V2X_BRAKE_RESPONSE",
                        "vehicle_id": vehicle_id,
                        "time": simulation_time,
                    }
                    events.append(response_event)
                    if on_event is not None:
                        on_event(response_event)
                del pending_responses[vehicle_id]

            controller.step()
            collision_pairs.update(controller.collision_pairs())
            frame_vehicles = [
                controller.snapshot(vehicle_id).__dict__
                for vehicle_id in controller.vehicle_ids()
            ]
            lane_groups = {}
            for vehicle in frame_vehicles:
                lane_groups.setdefault(vehicle["lane_id"], []).append(vehicle)
            for lane_vehicles in lane_groups.values():
                lane_vehicles.sort(key=lambda vehicle: vehicle["position"])
                for follower, leader in zip(
                    lane_vehicles,
                    lane_vehicles[1:],
                ):
                    gap = (
                        leader["position"]
                        - leader["length"]
                        - follower["position"]
                    )
                    minimum_separation_m = (
                        gap
                        if minimum_separation_m is None
                        else min(minimum_separation_m, gap)
                    )
                    closing_speed = follower["speed"] - leader["speed"]
                    if closing_speed > 0 and gap > 0:
                        ttc = gap / closing_speed
                        minimum_ttc_s = (
                            ttc
                            if minimum_ttc_s is None
                            else min(minimum_ttc_s, ttc)
                        )
            frame = {
                "time": controller.time,
                "vehicles": frame_vehicles,
                "rsu": {
                    "unit_id": settings.roadside_unit.unit_id,
                    "road_id": settings.roadside_unit.road_id,
                    "position": settings.roadside_unit.position,
                    "communication_range_m": (
                        settings.roadside_unit.communication_range_m
                    ),
                },
                "communication_mode": communication_mode.value,
            }
            observed_vehicle_ids.update(
                vehicle["vehicle_id"] for vehicle in frame_vehicles
            )
            history.append(frame)
            frame_vehicle_ids = {
                vehicle["vehicle_id"] for vehicle in frame_vehicles
            }
            if (
                initial_conditions is None
                and EXPECTED_VEHICLE_IDS.issubset(frame_vehicle_ids)
            ):
                initial_conditions = {
                    "time": frame["time"],
                    "vehicles": deepcopy(frame_vehicles),
                    "random_seed": settings.random_seed,
                }
            if on_step is not None:
                on_step(frame)

            if controller.min_expected_number == 0:
                break

        if gui and gui_end_delay:
            time.sleep(gui_end_delay)

    missing_vehicles = EXPECTED_VEHICLE_IDS - observed_vehicle_ids
    if not stopped and missing_vehicles:
        raise RuntimeError(
            f"SUMO scenario did not load expected vehicles: {sorted(missing_vehicles)}"
        )
    if not stopped and not emergency_event_sent:
        raise RuntimeError("The scheduled emergency-braking event did not occur")

    metrics = calculate_sumo_metrics(
        messages_transmitted=channel.messages_transmitted,
        transmission_attempts=channel.transmission_attempts,
        messages_received=channel.messages_received,
        latency_samples_ms=channel.latency_samples_ms,
        warning_records=warning_records,
        response_vehicle_ids=response_vehicle_ids,
        collision_pairs=collision_pairs,
        minimum_separation_m=minimum_separation_m,
        minimum_ttc_s=minimum_ttc_s,
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
        "stopped": stopped,
        "communication_mode": communication_mode.value,
        "initial_conditions": initial_conditions,
    }


def run_sumo_comparison(
    settings: SimulationSettings | None = None,
    *,
    modes: tuple[CommunicationMode | str, ...] = tuple(CommunicationMode),
    on_step: StepCallback | None = None,
    on_event: EventCallback | None = None,
    should_continue: ContinueCallback | None = None,
) -> dict[str, object]:
    settings = settings or SimulationSettings()
    comparison_results = {}
    reference_initial_conditions = None
    stopped = False

    for mode_value in modes:
        mode = CommunicationMode(mode_value)
        if should_continue is not None and not should_continue():
            stopped = True
            break

        def publish_step(frame, scenario_mode=mode):
            if on_step is not None:
                scenario_frame = dict(frame)
                scenario_frame["communication_mode"] = scenario_mode.value
                on_step(scenario_frame)

        def publish_event(event, scenario_mode=mode):
            if on_event is not None:
                scenario_event = dict(event)
                scenario_event["communication_mode"] = scenario_mode.value
                on_event(scenario_event)

        result = run_sumo_simulation(
            settings,
            communication_mode=mode,
            on_step=publish_step,
            on_event=publish_event,
            should_continue=should_continue,
        )
        comparison_results[mode.value] = result
        initial_conditions = result["initial_conditions"]
        if reference_initial_conditions is None:
            reference_initial_conditions = initial_conditions
        elif initial_conditions != reference_initial_conditions:
            raise RuntimeError(
                f"Initial conditions differed in {mode.value} comparison run"
            )
        if result["stopped"]:
            stopped = True
            break

    return {
        "stopped": stopped,
        "comparison_results": comparison_results,
        "initial_conditions": reference_initial_conditions,
        "random_seed": settings.random_seed,
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the SUMO + TraCI V2X collision-avoidance simulation."
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="run SUMO-GUI under TraCI control instead of headless SUMO",
    )
    parser.add_argument(
        "--gui-end-delay",
        type=float,
        default=45.0,
        metavar="SECONDS",
        help="keep the GUI connected and open this long after the simulation "
        "ends (default: 45; use 0 to close immediately)",
    )
    parser.add_argument(
        "--gui-step-delay",
        type=int,
        default=300,
        metavar="MILLISECONDS",
        help="minimum GUI delay between simulation steps in milliseconds "
        "(default: 300; affects visual playback, not simulated time)",
    )
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_argument_parser().parse_args(argv)
    results = run_sumo_simulation(
        gui=args.gui,
        gui_end_delay=args.gui_end_delay,
        gui_step_delay_ms=args.gui_step_delay,
    )
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
