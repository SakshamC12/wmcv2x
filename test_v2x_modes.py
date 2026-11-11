import unittest
from pathlib import Path

from sumo_simulation import (
    SimulationSettings,
    run_sumo_comparison,
    run_sumo_simulation,
)
from v2x_communication import (
    CommunicationMode,
    ControlledV2XChannel,
    RoadsideUnit,
)
from sumo_controller import VehicleSnapshot


class V2XModesTest(unittest.TestCase):
    def setUp(self):
        self.settings = SimulationSettings(end_time=10.0)

    def test_all_modes_share_deterministic_initial_conditions(self):
        result = run_sumo_comparison(self.settings)
        runs = result["comparison_results"]

        self.assertEqual(
            set(runs),
            {mode.value for mode in CommunicationMode},
        )
        initial_conditions = [
            runs[mode.value]["initial_conditions"]
            for mode in CommunicationMode
        ]
        self.assertTrue(
            all(item == initial_conditions[0] for item in initial_conditions)
        )
        self.assertEqual(initial_conditions[0]["random_seed"], 42)
        self.assertEqual(len(initial_conditions[0]["vehicles"]), 7)
        self.assertEqual(result["random_seed"], 42)

    def test_disabled_mode_sends_no_safety_messages_or_responses(self):
        result = run_sumo_simulation(
            self.settings,
            communication_mode=CommunicationMode.DISABLED,
        )

        self.assertEqual(result["events"][0]["time"], 5.0)
        self.assertEqual(result["message_log"], [])
        self.assertEqual(result["responded_vehicle_ids"], [])
        self.assertEqual(result["metrics"]["messages_transmitted"], 0)
        self.assertEqual(result["metrics"]["transmission_attempts"], 0)
        self.assertEqual(result["metrics"]["collisions"], 0)

    def test_v2v_only_preserves_delivery_and_response_timing(self):
        result = run_sumo_simulation(
            self.settings,
            communication_mode=CommunicationMode.V2V,
        )

        self.assertEqual(result["metrics"]["messages_transmitted"], 1)
        self.assertEqual(result["metrics"]["transmission_attempts"], 6)
        self.assertEqual(result["metrics"]["messages_received"], 6)
        self.assertEqual(result["metrics"]["vehicles_responding"], 3)
        self.assertEqual(
            [
                event["time"]
                for event in result["events"]
                if event["event_type"] == "V2X_BRAKE_RESPONSE"
            ],
            [5.5, 5.5, 5.5],
        )
        self.assertTrue(
            all(
                message["communication_type"] == "V2V"
                for message in result["message_log"]
            )
        )

    def test_v2i_only_logs_rsu_reception_and_broadcast(self):
        events = []
        result = run_sumo_simulation(
            self.settings,
            communication_mode=CommunicationMode.V2I,
            on_event=events.append,
        )

        self.assertEqual(result["metrics"]["messages_transmitted"], 2)
        self.assertEqual(result["metrics"]["transmission_attempts"], 7)
        self.assertEqual(result["metrics"]["vehicles_responding"], 3)
        self.assertEqual(
            [event["time"] for event in result["events"]
             if event["event_type"] == "V2X_BRAKE_RESPONSE"],
            [5.5, 5.5, 5.5],
        )
        self.assertTrue(
            any(event["event_type"] == "RSU_MESSAGE_RECEIVED" for event in events)
        )
        self.assertTrue(
            any(event["event_type"] == "RSU_PROCESSED_WARNING" for event in events)
        )
        self.assertTrue(
            any(
                event["event_type"] == "V2X_TRANSMISSION"
                and event["communication_type"] == "V2I"
                for event in events
            )
        )
        self.assertTrue(
            any(
                event["event_type"] == "V2X_TRANSMISSION"
                and event["communication_type"] == "I2V"
                for event in events
            )
        )
        self.assertTrue(
            all(
                message["communication_type"] == "I2V"
                for message in result["message_log"]
            )
        )

    def test_combined_mode_keeps_v2v_and_adds_rsu_delivery(self):
        events = []
        result = run_sumo_simulation(
            self.settings,
            communication_mode=CommunicationMode.COMBINED,
            on_event=events.append,
        )
        message_types = {
            message["communication_type"]
            for message in result["message_log"]
        }

        self.assertEqual(message_types, {"V2V", "I2V"})
        self.assertEqual(result["metrics"]["messages_transmitted"], 3)
        self.assertEqual(result["metrics"]["transmission_attempts"], 13)
        self.assertEqual(result["metrics"]["vehicles_responding"], 3)
        self.assertEqual(
            [event["time"] for event in result["events"]
             if event["event_type"] == "V2X_BRAKE_RESPONSE"],
            [5.5, 5.5, 5.5],
        )

    def test_rsu_broadcast_delivers_only_inside_its_range(self):
        channel = ControlledV2XChannel(communication_range_m=100.0)
        rsu = RoadsideUnit(position=850.0, communication_range_m=100.0)
        vehicles = (
            VehicleSnapshot(
                "inside", 800.0, 20.0, "highway", "highway_0", 5.0, "passenger"
            ),
            VehicleSnapshot(
                "outside", 700.0, 20.0, "highway", "highway_1", 5.0, "passenger"
            ),
            VehicleSnapshot(
                "other_road", 820.0, 20.0, "other", "other_0", 5.0, "passenger"
            ),
        )
        message = {
            "message_id": "test-message",
            "sender_id": rsu.unit_id,
            "position": 900.0,
            "speed": 0.0,
            "road_id": "highway",
            "lane_id": "highway_0",
            "vehicle_length": 12.0,
        }

        deliveries = channel.broadcast(
            message,
            vehicles,
            simulation_time=5.0,
            communication_type="I2V",
            transmitter_position=rsu.position,
        )

        self.assertEqual([delivery.recipient_id for delivery in deliveries], ["inside"])
        self.assertEqual(deliveries[0].message["warning_distance_m"], 100.0)
        self.assertEqual(deliveries[0].message["communication_type"], "I2V")
        self.assertEqual(deliveries[0].message["transmitted_at"], 5.0)
        self.assertEqual(deliveries[0].message["received_at"], 5.0)
        self.assertEqual(channel.transmission_attempts, 1)
        self.assertEqual(channel.messages_received, 1)

    def test_out_of_range_truck_does_not_reach_rsu(self):
        events = []
        settings = SimulationSettings(
            end_time=10.0,
            roadside_unit=RoadsideUnit(
                position=1200.0,
                communication_range_m=10.0,
            ),
        )
        result = run_sumo_simulation(
            settings,
            communication_mode=CommunicationMode.V2I,
            on_event=events.append,
        )

        self.assertEqual(result["metrics"]["messages_transmitted"], 0)
        self.assertEqual(result["metrics"]["messages_received"], 0)
        self.assertFalse(
            any(event["event_type"] == "RSU_MESSAGE_RECEIVED" for event in events)
        )

    def test_comparison_keeps_baseline_outcome_unforced(self):
        result = run_sumo_comparison(self.settings)
        baseline = result["comparison_results"]["disabled"]
        self.assertIsInstance(baseline["metrics"]["collisions"], int)
        self.assertGreaterEqual(baseline["metrics"]["collisions"], 0)
        self.assertIn("minimum_vehicle_separation_m", baseline["metrics"])
        self.assertIn("minimum_ttc_s", baseline["metrics"])
        self.assertFalse(baseline["message_log"])

    def test_braked_vehicles_remain_stopped_in_every_mode(self):
        settings = SimulationSettings(end_time=15.0)
        for mode in CommunicationMode:
            with self.subTest(mode=mode.value):
                frames = []
                result = run_sumo_simulation(
                    settings,
                    communication_mode=mode,
                    on_step=frames.append,
                )
                braking_times = {
                    event["vehicle_id"]: event["time"]
                    for event in result["events"]
                    if event["event_type"] == "V2X_BRAKE_RESPONSE"
                }
                braking_times["emergency_truck"] = 5.0
                self.assertEqual(
                    set(braking_times) - {"emergency_truck"},
                    set(result["responded_vehicle_ids"]),
                )

                for vehicle_id, braking_time in braking_times.items():
                    samples = [
                        next(
                            vehicle["speed"]
                            for vehicle in frame["vehicles"]
                            if vehicle["vehicle_id"] == vehicle_id
                        )
                        for frame in frames
                        if frame["time"] >= braking_time + 5.0
                    ]
                    self.assertTrue(samples, vehicle_id)
                    self.assertTrue(
                        any(speed <= 0.05 for speed in samples),
                        f"{mode.value}: {vehicle_id} never stopped",
                    )
                    self.assertTrue(
                        all(speed <= 0.05 for speed in samples),
                        f"{mode.value}: {vehicle_id} accelerated after stopping",
                    )


if __name__ == "__main__":
    unittest.main()
