import unittest

from sumo_simulation import SimulationSettings, run_sumo_simulation


class SumoV2XIntegrationTest(unittest.TestCase):
    def test_multi_vehicle_emergency_event_and_v2x_responses(self):
        results = run_sumo_simulation(
            SimulationSettings(end_time=10.0)
        )
        metrics = results["metrics"]

        self.assertEqual(len(results["vehicle_ids"]), 7)
        self.assertEqual(
            [event["time"] for event in results["events"]
             if event["event_type"] == "EMERGENCY_BRAKING"],
            [5.0],
        )
        self.assertEqual(
            [event["time"] for event in results["events"]
             if event["event_type"] == "V2X_BRAKE_RESPONSE"],
            [5.5, 5.5, 5.5],
        )
        self.assertEqual(metrics["messages_transmitted"], 1)
        self.assertEqual(metrics["transmission_attempts"], 6)
        self.assertEqual(metrics["messages_received"], 6)
        self.assertEqual(metrics["packet_delivery_ratio_percent"], 100.0)
        self.assertEqual(metrics["vehicles_responding"], 3)
        self.assertEqual(metrics["collisions"], 0)
        self.assertEqual(
            metrics["communication_latency_ms"],
            0.0,
        )
        self.assertFalse(metrics["wireless_measurement"])
        self.assertTrue(
            all(
                event["event_type"] == "V2X_BRAKE_RESPONSE"
                for event in results["events"][1:]
            )
        )


if __name__ == "__main__":
    unittest.main()
