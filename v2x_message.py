
import uuid
import json


def create_safety_message(vehicle, simulation_time):
    """Create a simplified DENM-like emergency notification."""

    message = {
        "message_id": str(uuid.uuid4()),
        "sender_id": vehicle.vehicle_id,
        "event_type": "EMERGENCY_BRAKING",
        "position": round(vehicle.position, 2),
        "speed": round(vehicle.speed, 2),
        "timestamp": round(simulation_time, 2),
        "severity": "HIGH"
    }

    return message


def transmit_message(message):
    """Serialize the message into JSON for transmission."""

    return json.dumps(message)


def receive_message(transmitted_data):
    """Decode the received JSON message."""

    return json.loads(transmitted_data)


def calculate_ttc(distance, closing_speed):
    """Calculate estimated Time-to-Collision."""

    if closing_speed <= 0:
        return None

    return distance / closing_speed
