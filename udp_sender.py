
import socket

from vehicle import Vehicle
from v2x_message import (
    create_safety_message,
    transmit_message
)

HOST = "127.0.0.1"
PORT = 5005

# Create Vehicle A
truck = Vehicle("A", "Truck", 100, 20)

# Simulate emergency braking
truck.brake(8.0)

# Generate a safety message
message = create_safety_message(truck, 2.0)

# Convert message into JSON
message_data = transmit_message(message)

# Create UDP sender socket
sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Send the message
sender.sendto(
    message_data.encode("utf-8"),
    (HOST, PORT)
)

print("VEHICLE A — UDP SENDER")
print("----------------------")
print("Emergency braking detected.")
print("V2X safety message transmitted.")
print("Destination:", HOST, PORT)
print("Message:", message_data)

sender.close()
