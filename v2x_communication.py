from dataclasses import dataclass
from enum import Enum
from typing import Mapping

from sumo_controller import VehicleSnapshot
from v2x_message import receive_message, transmit_message


@dataclass(frozen=True)
class Delivery:
    recipient_id: str
    message: dict[str, object]


class CommunicationMode(str, Enum):
    DISABLED = "disabled"
    V2V = "v2v"
    V2I = "v2i"
    COMBINED = "combined"

    @property
    def enables_v2v(self) -> bool:
        return self in (CommunicationMode.V2V, CommunicationMode.COMBINED)

    @property
    def enables_v2i(self) -> bool:
        return self in (CommunicationMode.V2I, CommunicationMode.COMBINED)


@dataclass(frozen=True)
class RoadsideUnit:
    unit_id: str = "rsu_01"
    road_id: str = "highway"
    position: float = 850.0
    communication_range_m: float = 200.0

    def __post_init__(self):
        if self.communication_range_m < 0:
            raise ValueError("RSU communication range must be non-negative")


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
        *,
        communication_type: str = "V2V",
        transmitter_position: float | None = None,
    ) -> tuple[Delivery, ...]:
        serialized_message = transmit_message(dict(message))
        self.messages_transmitted += 1
        deliveries = []
        excluded_vehicle_ids = {
            str(message["sender_id"]),
            str(message.get("source_vehicle_id", "")),
        }

        for vehicle in vehicles:
            if vehicle.vehicle_id in excluded_vehicle_ids:
                continue
            if vehicle.road_id != message["road_id"]:
                continue

            distance = abs(
                vehicle.position
                - (
                    float(message["position"])
                    if transmitter_position is None
                    else transmitter_position
                )
            )
            if distance > self.communication_range_m:
                continue

            self.transmission_attempts += 1
            received_message = receive_message(serialized_message)
            received_message["receiver_id"] = vehicle.vehicle_id
            received_message["communication_type"] = communication_type
            received_message["transmitted_at"] = simulation_time
            received_message["received_at"] = simulation_time
            received_message["communication_latency_ms"] = 0.0
            received_message["warning_distance_m"] = round(
                abs(vehicle.position - float(message["position"])),
                2,
            )
            self.messages_received += 1
            self.latency_samples_ms.append(0.0)
            deliveries.append(Delivery(vehicle.vehicle_id, received_message))

        return tuple(deliveries)

    def unicast(
        self,
        message: Mapping[str, object],
        receiver_id: str,
        simulation_time: float,
        *,
        communication_type: str,
        warning_distance_m: float,
    ) -> Delivery:
        serialized_message = transmit_message(dict(message))
        self.messages_transmitted += 1
        self.transmission_attempts += 1
        received_message = receive_message(serialized_message)
        received_message["receiver_id"] = receiver_id
        received_message["communication_type"] = communication_type
        received_message["transmitted_at"] = simulation_time
        received_message["received_at"] = simulation_time
        received_message["communication_latency_ms"] = 0.0
        received_message["warning_distance_m"] = round(warning_distance_m, 2)
        self.messages_received += 1
        self.latency_samples_ms.append(0.0)
        return Delivery(receiver_id, received_message)
