from dataclasses import dataclass
from typing import Mapping

from sumo_controller import VehicleSnapshot
from v2x_message import receive_message, transmit_message


@dataclass(frozen=True)
class Delivery:
    recipient_id: str
    message: dict[str, object]


class ControlledV2XChannel:
    """In-process message delivery with zero modeled delay and no packet loss."""

    def __init__(self, communication_range_m: float):
        if communication_range_m < 0:
            raise ValueError("communication_range_m must be non-negative")
        self.communication_range_m = communication_range_m
        self.messages_transmitted = 0
        self.transmission_attempts = 0
        self.messages_received = 0
        self.latency_samples_ms: list[float] = []

    def broadcast(
        self,
        message: Mapping[str, object],
        vehicles: tuple[VehicleSnapshot, ...],
        simulation_time: float,
    ) -> tuple[Delivery, ...]:
        serialized_message = transmit_message(dict(message))
        self.messages_transmitted += 1
        deliveries = []

        for vehicle in vehicles:
            if vehicle.vehicle_id == message["sender_id"]:
                continue
            if vehicle.road_id != message["road_id"]:
                continue

            distance = abs(vehicle.position - float(message["position"]))
            if distance > self.communication_range_m:
                continue

            self.transmission_attempts += 1
            received_message = receive_message(serialized_message)
            received_message["receiver_id"] = vehicle.vehicle_id
            received_message["received_at"] = simulation_time
            received_message["communication_latency_ms"] = 0.0
            received_message["warning_distance_m"] = round(distance, 2)
            self.messages_received += 1
            self.latency_samples_ms.append(0.0)
            deliveries.append(Delivery(vehicle.vehicle_id, received_message))

        return tuple(deliveries)
