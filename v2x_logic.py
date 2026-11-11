from dataclasses import dataclass
from math import sqrt
from typing import Mapping

from sumo_controller import VehicleSnapshot
from v2x_message import calculate_ttc


@dataclass(frozen=True)
class RiskAssessment:
    distance_m: float
    same_lane: bool
    ttc_seconds: float | None
    should_brake: bool
    reason: str


def assess_collision_risk(
    message: Mapping[str, object],
    receiver: VehicleSnapshot,
    maximum_ttc_seconds: float,
) -> RiskAssessment:
    sender_position = float(message["position"])
    sender_speed = float(message["speed"])
    distance = abs(sender_position - receiver.position)
    same_lane = (
        message["road_id"] == receiver.road_id
        and message["lane_id"] == receiver.lane_id
    )

    if not same_lane:
        return RiskAssessment(
            distance, False, None, False, "emergency vehicle is in another lane"
        )

    gap = sender_position - receiver.position - float(message["vehicle_length"])
    if gap <= 0:
        return RiskAssessment(distance, True, 0.0, True, "no remaining gap")

    braking_duration = float(message.get("emergency_braking_duration_s", 0.0))
    closing_speed = receiver.speed - sender_speed
    if braking_duration <= 0 or sender_speed <= 0:
        ttc = calculate_ttc(gap, closing_speed)
    else:
        braking_deceleration = sender_speed / braking_duration
        discriminant = (
            closing_speed * closing_speed
            + 2 * braking_deceleration * gap
        )
        braking_ttc = (
            -closing_speed + sqrt(discriminant)
        ) / braking_deceleration
        if braking_ttc <= braking_duration:
            ttc = braking_ttc
        else:
            distance_closed_during_braking = (
                closing_speed * braking_duration
                + 0.5 * braking_deceleration * braking_duration**2
            )
            remaining_gap = gap - distance_closed_during_braking
            ttc = (
                braking_duration + remaining_gap / receiver.speed
                if receiver.speed > 0
                else None
            )

    if ttc is None:
        return RiskAssessment(
            distance,
            True,
            None,
            False,
            "receiver is not closing on the emergency vehicle",
        )
    if ttc > maximum_ttc_seconds:
        return RiskAssessment(
            distance, True, ttc, False, "time-to-collision is outside the warning threshold"
        )

    return RiskAssessment(distance, True, ttc, True, "closing risk within warning threshold")
