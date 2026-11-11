from collections.abc import Iterable, Sequence


def calculate_sumo_metrics(
    messages_transmitted: int,
    transmission_attempts: int,
    messages_received: int,
    latency_samples_ms: Sequence[float],
    warning_records: Sequence[dict[str, object]],
    response_vehicle_ids: Iterable[str],
    collision_pairs: Iterable[tuple[str, str]],
) -> dict[str, object]:
    packet_delivery_ratio = (
        messages_received / transmission_attempts * 100
        if transmission_attempts
        else 0.0
    )
    warning_distances = [
        float(record["warning_distance_m"])
        for record in warning_records
    ]
    warning_ttcs = [
        float(record["ttc_at_warning_s"])
        for record in warning_records
        if record["ttc_at_warning_s"] is not None
    ]
    unique_collision_pairs = {tuple(sorted(pair)) for pair in collision_pairs}
    collision_vehicle_ids = {
        vehicle_id
        for pair in unique_collision_pairs
        for vehicle_id in pair
    }
    response_ids = set(response_vehicle_ids)

    return {
        "messages_transmitted": messages_transmitted,
        "transmission_attempts": transmission_attempts,
        "messages_received": messages_received,
        "packet_delivery_ratio_percent": round(packet_delivery_ratio, 2),
        "communication_latency_ms": (
            round(sum(latency_samples_ms) / len(latency_samples_ms), 4)
            if latency_samples_ms
            else None
        ),
        "warning_distance_m": (
            round(sum(warning_distances) / len(warning_distances), 2)
            if warning_distances
            else None
        ),
        "ttc_at_warning_s": (
            round(sum(warning_ttcs) / len(warning_ttcs), 2)
            if warning_ttcs
            else None
        ),
        "vehicles_responding": len(response_ids),
        "collisions": len(unique_collision_pairs),
        "vehicles_avoiding_collisions": len(
            response_ids - collision_vehicle_ids
        ),
        "collision_vehicle_ids": sorted(collision_vehicle_ids),
        "communication_model": "in-process; zero modeled delay; no packet loss",
        "transmission_count_semantics": (
            "messages_transmitted counts broadcasts; transmission_attempts "
            "counts one recipient-directed copy per eligible nearby vehicle"
        ),
        "communication_latency_semantics": (
            "Modeled delay only; not wall-clock processing time or radio latency"
        ),
        "warning_distance_semantics": (
            "Mean front-position separation at message receipt"
        ),
        "collision_avoidance_semantics": (
            "Responders without a recorded collision; not a counterfactual "
            "proof that braking prevented a collision"
        ),
        "wireless_measurement": False,
    }