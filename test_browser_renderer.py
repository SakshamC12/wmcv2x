import unittest
from pathlib import Path

from streamlit.testing.v1 import AppTest


class CommandCenterRenderingTest(unittest.TestCase):
    def test_command_center_renders_without_streamlit_exception(self):
        app = AppTest.from_file(
            str(Path(__file__).with_name("app.py")),
            default_timeout=15,
        ).run()
        app.sidebar.radio(key="application_view").set_value(
            "V2X Command Center"
        ).run()

        self.assertFalse(app.exception)
        self.assertIn(
            "Live SUMO Highway",
            [heading.value for heading in app.subheader],
        )


if __name__ == "__main__":
    unittest.main()
