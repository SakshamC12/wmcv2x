
from vehicle import Vehicle

truck = Vehicle("A", "Truck", 100, 20)

print("Before braking:")
print("Position:", truck.position)
print("Speed:", truck.speed)

truck.brake()

for second in range(3):
    truck.update(1)

    print(f"\nAfter {second + 1} second(s):")
    print("Position:", round(truck.position, 2))
    print("Speed:", round(truck.speed, 2))
