import contextlib
import io
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import call, patch

from sumo_controller import SumoController
from sumo_simulation import main


class SumoCommandLineTest(unittest.TestCase):
    def _assert_controller_uses_binary(
        self,
        gui: bool,
        gui_step_delay_ms: int = 300,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            sumo_home = Path(temporary_directory)
            binary_name = "sumo-gui.exe" if gui else "sumo.exe"
            binary = sumo_home / "bin" / binary_name
            binary.parent.mkdir()
            binary.touch()

            with (
                patch.dict(os.environ, {"SUMO_HOME": str(sumo_home)}),
                patch("sumo_controller.traci.start") as start,
                patch("sumo_controller.traci.close") as close,
                patch("sumo_controller.traci.gui.setOffset") as set_offset,
                patch("sumo_controller.traci.gui.setZoom") as set_zoom,
            ):
                with SumoController(
                    Path("scenario.sumocfg"),
                    gui=gui,
                    gui_step_delay_ms=gui_step_delay_ms,
                ):
                    command = start.call_args.args[0]
                    self.assertEqual(command[0], str(binary))
                    self.assertIn("-c", command)
                    if gui:
                        delay_index = command.index("--delay")
                        self.assertEqual(
                            command[delay_index + 1],
                            str(gui_step_delay_ms),
                        )
                        set_offset.assert_called_once_with(
                            "View #0", 700.0, 0.0
                        )
                        set_zoom.assert_called_once_with("View #0", 350.0)
                    else:
                        self.assertNotIn("--delay", command)
                        set_offset.assert_not_called()
                        set_zoom.assert_not_called()
                close.assert_called_once_with()

    def test_headless_mode_selects_sumo_executable(self):
        self._assert_controller_uses_binary(gui=False)

    def test_gui_mode_selects_sumo_gui_executable(self):
        self._assert_controller_uses_binary(gui=True)

    def test_gui_controller_defaults_to_presentation_delay(self):
        controller = SumoController(Path("scenario.sumocfg"), gui=True)
        self.assertEqual(controller.gui_step_delay_ms, 300)

    def test_gui_delay_is_configurable(self):
        self._assert_controller_uses_binary(
            gui=True,
            gui_step_delay_ms=500,
        )

    def test_gui_sets_presentation_camera_and_vehicle_colors(self):
        with (
            patch("sumo_controller.traci.gui.trackVehicle") as track_vehicle,
            patch("sumo_controller.traci.vehicle.setColor") as set_color,
        ):
            controller = SumoController(Path("scenario.sumocfg"), gui=True)
            controller.track_vehicle("emergency_truck")
            controller.set_presentation_color("emergency_truck")
            controller.set_presentation_color("car_01")

        track_vehicle.assert_called_once_with("View #0", "emergency_truck")
        self.assertEqual(
            set_color.call_args_list,
            [
                call("emergency_truck", (220, 35, 35, 255)),
                call("car_01", (0, 150, 255, 255)),
            ],
        )

    def test_help_lists_options_without_starting_simulation(self):
        output = io.StringIO()
        with (
            contextlib.redirect_stdout(output),
            patch("sumo_simulation.run_sumo_simulation") as run_simulation,
            self.assertRaises(SystemExit) as exit_context,
        ):
            main(["--help"])

        self.assertEqual(exit_context.exception.code, 0)
        self.assertIn("--gui", output.getvalue())
        self.assertIn("--gui-end-delay", output.getvalue())
        self.assertIn("--gui-step-delay", output.getvalue())
        self.assertIn("default: 300", output.getvalue())
        self.assertIn("default: 45", output.getvalue())
        run_simulation.assert_not_called()

    def test_headless_cli_json_shape_is_unchanged(self):
        fake_results = {
            "vehicle_ids": ["emergency_truck", "car_01"],
            "responded_vehicle_ids": ["car_01"],
            "events": [{"event_type": "EMERGENCY_BRAKING", "time": 5.0}],
            "metrics": {"messages_received": 1},
        }
        output = io.StringIO()
        with (
            contextlib.redirect_stdout(output),
            patch("sumo_simulation.run_sumo_simulation", return_value=fake_results)
            as run_simulation,
        ):
            main([])

        run_simulation.assert_called_once_with(
            gui=False,
            gui_end_delay=45.0,
            gui_step_delay_ms=300,
        )
        self.assertEqual(
            json.loads(output.getvalue()),
            {
                "vehicle_ids": fake_results["vehicle_ids"],
                "responded_vehicle_ids": fake_results["responded_vehicle_ids"],
                "events": fake_results["events"],
                "metrics": fake_results["metrics"],
            },
        )

    def test_gui_cli_accepts_presentation_delay_options(self):
        with (
            patch("sumo_simulation.run_sumo_simulation") as run_simulation,
            contextlib.redirect_stdout(io.StringIO()),
        ):
            run_simulation.return_value = {
                "vehicle_ids": [],
                "responded_vehicle_ids": [],
                "events": [],
                "metrics": {},
            }
            main(
                [
                    "--gui",
                    "--gui-step-delay",
                    "500",
                    "--gui-end-delay",
                    "60",
                ]
            )

        run_simulation.assert_called_once_with(
            gui=True,
            gui_end_delay=60.0,
            gui_step_delay_ms=500,
        )


if __name__ == "__main__":
    unittest.main()
