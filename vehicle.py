
class Vehicle:
    def __init__(self, vehicle_id, vehicle_type, position, speed):
        self.vehicle_id = vehicle_id
        self.vehicle_type = vehicle_type

        # Position along the road, in metres
        self.position = position

        # Speed in metres per second
        self.speed = speed

        # Acceleration in metres per second squared
        self.acceleration = 0.0

        self.is_braking = False

    def brake(self, braking_force=8.0):
        """Apply braking to the vehicle."""
        self.acceleration = -braking_force
        self.is_braking = True

    def update(self, time_step):
        """Update vehicle position and speed."""

        old_speed = self.speed

        # Update speed based on acceleration
        self.speed += self.acceleration * time_step

        # A vehicle cannot travel backwards in this model
        self.speed = max(0.0, self.speed)

        # Use average speed to update position
        average_speed = (old_speed + self.speed) / 2
        self.position += average_speed * time_step
