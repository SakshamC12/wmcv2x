import shutil
import unittest
from pathlib import Path

import traci


SCENARIO_DIR = Path(__file__).parent / "sumo_scenario"


class SumoTraCISmokeTest(unittest.TestCase):
    def test_vehicle_moves_under_traci_control(self):
        sumo_binary = shutil.which("sumo")
        if sumo_binary is None:
            self.fail(
                "SUMO was not found on PATH. Add SUMO_HOME\\bin to PATH "
                "or set PATH for this shell before running the test."
            )

        config_file = SCENARIO_DIR / "smoke.sumocfg"
        started = False

        try:
            traci.start(
                [
                    sumo_binary,
                    "-c",
                    str(config_file),
                    "--no-step-log",
                    "true",
                    "--duration-log.disable",
                    "true",
                ]
            )
            started = True

            for _ in range(10):
                traci.simulationStep()

            self.assertEqual(traci.simulation.getTime(), 1.0)
            self.assertIn("smoke_car", traci.vehicle.getIDList())
            self.assertGreater(traci.vehicle.getSpeed("smoke_car"), 0)
            self.assertGreater(traci.vehicle.getPosition("smoke_car")[0], 0)
        finally:
            if started:
                traci.close()


if __name__ == "__main__":
    unittest.main()
