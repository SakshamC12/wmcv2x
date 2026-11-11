
import socket
import json

HOST = "127.0.0.1"
PORT = 5005

# Create a UDP socket
receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Listen for incoming messages
receiver.bind((HOST, PORT))

print("VEHICLE B — UDP RECEIVER")
print("------------------------")
print(f"Listening on {HOST}:{PORT}")
print("Waiting for V2X messages...\n")

try:
    while True:

        # Receive a datagram (maximum 4096 bytes)
        data, address = receiver.recvfrom(4096)

        # Decode JSON
        message = json.loads(data.decode("utf-8"))

        print("V2X MESSAGE RECEIVED")
        print("--------------------")
        print("Sender address:", address)
        print("Message ID:", message["message_id"])
        print("Vehicle:", message["sender_id"])
        print("Event:", message["event_type"])
        print("Position:", message["position"], "m")
        print("Speed:", message["speed"], "m/s")
        print("Timestamp:", message["timestamp"], "s")
        print("Severity:", message["severity"])
        print()

except KeyboardInterrupt:
    print("\nReceiver stopped.")

finally:
    receiver.close()
