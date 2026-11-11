
import socket
import json
import time

from vehicle import Vehicle
from v2x_message import (
    create_safety_message,
    transmit_message,
    calculate_ttc
)


def run_simulation(
    enable_driver_response=False,
    response_delay=1.0
):
    # ----------------------------------------------
    # CREATE VEHICLES
    # ----------------------------------------------

    truck = Vehicle("A", "Truck", 100, 20)
    car = Vehicle("B", "Car", 0, 25)

    # ----------------------------------------------
    # UDP COMMUNICATION SETUP
    # ----------------------------------------------

    HOST = "127.0.0.1"
    PORT = 5005

    receiver_socket = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    )

    sender_socket = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    )

    receiver_socket.bind((HOST, PORT))
    receiver_socket.settimeout(1.0)

    # ----------------------------------------------
    # SIMULATION SETTINGS
    # ----------------------------------------------

    time_step = 0.1
    brake_time = 2.0
    max_simulation_time = 15

    history = []
    message_log = []

    message_sent = False
    message_received = False

    # Performance metric variables
    packets_sent = 0
    packets_received = 0
    latency_samples = []

    warning_distance = None
    warning_time = None

    driver_responded = False
    response_time = None

    collision_detected = False
    collision_time = None

    message_latency = None

    total_steps = int(max_simulation_time / time_step)

    try:

        for step in range(1, total_steps + 1):

            current_time = round(step * time_step, 2)

            # --------------------------------------
            # TRUCK EMERGENCY BRAKING
            # --------------------------------------

            if current_time >= brake_time and not truck.is_braking:

                truck.brake(8.0)

                print(
                    f"[{current_time:.1f}s] "
                    "TRUCK EMERGENCY BRAKING!"
                )

                # Generate V2X message
                outgoing_message = create_safety_message(
                    truck,
                    current_time
                )

                # Convert message to JSON
                message_data = transmit_message(
                    outgoing_message
                )

                # ----------------------------------
                # SEND UDP MESSAGE
                # ----------------------------------

                start_time = time.perf_counter_ns()

                sender_socket.sendto(
                    message_data.encode("utf-8"),
                    (HOST, PORT)
                )

                packets_sent += 1
                message_sent = True

                # ----------------------------------
                # RECEIVE UDP MESSAGE
                # ----------------------------------

                try:

                    received_data, sender_address = (
                        receiver_socket.recvfrom(4096)
                    )

                    end_time = time.perf_counter_ns()

                    message_latency = (
                        end_time - start_time
                    ) / 1_000_000

                    latency_samples.append(message_latency)

                    incoming_message = json.loads(
                        received_data.decode("utf-8")
                    )

                    incoming_message["latency_ms"] = (
                        message_latency
                    )

                    message_log.append(incoming_message)

                    packets_received += 1
                    message_received = True

                    print("\n--- UDP V2X TRANSMISSION ---")
                    print("Sender:", sender_address)
                    print("Vehicle:", incoming_message["sender_id"])
                    print("Event:", incoming_message["event_type"])
                    print("Message received successfully!")
                    print(
                        f"UDP latency: {message_latency:.4f} ms"
                    )

                except socket.timeout:

                    print("UDP message reception timed out.")

            # --------------------------------------
            # DRIVER RESPONSE
            # --------------------------------------

            if (
                enable_driver_response
                and message_received
                and not driver_responded
                and current_time >= brake_time + response_delay
            ):

                car.brake(6.0)

                driver_responded = True
                response_time = current_time

                print(
                    f"[{current_time:.1f}s] "
                    "CAR DRIVER RESPONSE: BRAKING!"
                )

            # --------------------------------------
            # UPDATE VEHICLES
            # --------------------------------------

            truck.update(time_step)
            car.update(time_step)

            separation = truck.position - car.position

            closing_speed = car.speed - truck.speed

            ttc = calculate_ttc(
                separation,
                closing_speed
            )

            # --------------------------------------
            # WARNING SYSTEM
            # --------------------------------------

            if separation <= 0:
                warning_level = "COLLISION"

            elif driver_responded and closing_speed <= 0:
                warning_level = "RISK MITIGATED"

            elif driver_responded:
                warning_level = "DRIVER RESPONDING"

            elif not message_received:
                warning_level = "NO WARNING"

            elif ttc is not None and ttc <= 3:
                warning_level = "CRITICAL"

            elif ttc is not None and ttc <= 6:
                warning_level = "HIGH"

            else:
                warning_level = "HAZARD DETECTED"

            # --------------------------------------
            # RECORD FIRST WARNING DISTANCE
            # --------------------------------------

            if (
                warning_distance is None
                and message_received
                and warning_level not in ["NO WARNING", "COLLISION"]
            ):

                warning_distance = round(separation, 2)
                warning_time = current_time

            # --------------------------------------
            # SAVE SIMULATION HISTORY
            # --------------------------------------

            history.append({
                "time": current_time,
                "truck_position": truck.position,
                "car_position": car.position,
                "truck_speed": truck.speed,
                "car_speed": car.speed,
                "separation": separation,
                "ttc": ttc,
                "warning_level": warning_level,
                "driver_responded": driver_responded
            })

            # --------------------------------------
            # COLLISION DETECTION
            # --------------------------------------

            if separation <= 0:

                collision_detected = True
                collision_time = current_time

                print(
                    f"[WARNING] COLLISION DETECTED "
                    f"AT {current_time:.1f}s!"
                )

                break

    finally:
        sender_socket.close()
        receiver_socket.close()

    # ----------------------------------------------
    # CALCULATE PERFORMANCE METRICS
    # ----------------------------------------------

    if packets_sent > 0:
        packet_delivery_ratio = (
            packets_received / packets_sent
        ) * 100
    else:
        packet_delivery_ratio = 0.0

    if latency_samples:
        average_latency = sum(latency_samples) / len(latency_samples)
        minimum_latency = min(latency_samples)
        maximum_latency = max(latency_samples)
    else:
        average_latency = None
        minimum_latency = None
        maximum_latency = None

    metrics = {
        "packets_sent": packets_sent,
        "packets_received": packets_received,
        "packet_delivery_ratio": round(packet_delivery_ratio, 2),
        "average_latency": (
            round(average_latency, 4)
            if average_latency is not None else None
        ),
        "minimum_latency": (
            round(minimum_latency, 4)
            if minimum_latency is not None else None
        ),
        "maximum_latency": (
            round(maximum_latency, 4)
            if maximum_latency is not None else None
        ),
        "warning_distance": warning_distance,
        "warning_time": warning_time
    }

    # ----------------------------------------------
    # RETURN RESULTS TO DASHBOARD
    # ----------------------------------------------

    return {
        "history": history,
        "message_log": message_log,
        "collision_detected": collision_detected,
        "collision_time": collision_time,
        "message_received": message_received,
        "message_sent": message_sent,
        "message_latency": message_latency,
        "driver_responded": driver_responded,
        "response_time": response_time,
        "driver_response_enabled": enable_driver_response,
        "metrics": metrics
    }
