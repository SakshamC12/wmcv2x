import unittest
from pathlib import Path
from queue import Queue

from streamlit.testing.v1 import AppTest

from simulation_worker import SimulationWorker, WorkerEvent, WorkerState
from sumo_simulation import SimulationSettings
from v2x_communication import CommunicationMode


class _CompletedWorker:
    def __init__(self, *, events=(), results=None, comparison=False):
        self.state = WorkerState.COMPLETED
        self.communication_mode = CommunicationMode.V2V
        self.settings = SimulationSettings(end_time=10.0)
        self.results = results or {
            "metrics": {
                "collisions": 0,
                "minimum_vehicle_separation_m": 18.5,
                "minimum_ttc_s": 2.4,
                "vehicles_responding": 3,
                "messages_transmitted": 1,
                "messages_received": 3,
                "transmission_attempts": 3,
                "packet_delivery_ratio_percent": 100.0,
                "communication_latency_ms": 0.0,
                "warning_distance_m": 25.0,
                "ttc_at_warning_s": 2.4,
                "vehicles_avoiding_collisions": 3,
            }
        }
        self.error = None
        self.telemetry_queue = Queue(maxsize=1)
        self.event_queue = Queue()
        self.comparison_modes = tuple(CommunicationMode) if comparison else None

    def reset(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def pause(self):
        pass

    def resume(self):
        pass


def _render_with_worker(worker, *, events, comparison=False):
    app = AppTest.from_file(
        str(Path(__file__).with_name("app.py")),
        default_timeout=15,
    ).run()
    app.sidebar.radio(key="application_view").set_value(
        "V2X Command Center"
    ).run()
    app.session_state["command_center_worker"] = worker
    app.session_state["command_center_telemetry"] = None
    app.session_state["command_center_events"] = list(events)
    app.session_state["command_center_comparison"] = comparison
    app.run()
    return app


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

    def test_safety_alerts_use_confirmed_events_and_keep_event_times(self):
        worker = SimulationWorker(SimulationSettings(end_time=10.0))
        worker.start()
        self.assertTrue(worker.join(timeout=30))
        self.assertEqual(worker.state, WorkerState.COMPLETED, worker.error)
        events = []
        while not worker.event_queue.empty():
            events.append(worker.event_queue.get_nowait())
        self.assertEqual(worker.results["metrics"]["collisions"], 0)
        self.assertFalse(
            any(event.event_type == "COLLISION_DETECTED" for event in events)
        )

        app = _render_with_worker(
            worker,
            events=events,
        )
        rendered_alerts = "\n".join(item.value for item in app.markdown)

        self.assertFalse(app.exception)
        self.assertIn("EMERGENCY BRAKING DETECTED", rendered_alerts)
        self.assertIn("Simulation time: 5.0 s", rendered_alerts)
        self.assertIn("CONNECTED VEHICLES RESPONDING", rendered_alerts)
        self.assertIn("at 5.5 s: car_01", rendered_alerts)
        self.assertIn("V2V WARNING BROADCAST", rendered_alerts)
        self.assertIn(
            "Intended recipients: 6; successful deliveries: 6",
            rendered_alerts,
        )
        self.assertNotIn("COLLISION DETECTED", rendered_alerts)
        self.assertIn("NO COLLISION RECORDED", rendered_alerts)
        self.assertIn("position:fixed", rendered_alerts)

    def test_confirmed_collision_event_shows_persistent_collision_alert(self):
        events = [
            WorkerEvent(
                "COLLISION_DETECTED",
                8.2,
                {
                    "vehicle_ids": ["car_01", "emergency_truck"],
                    "collision_count": 1,
                },
                3,
            )
        ]
        results = _CompletedWorker().results
        results["metrics"]["collisions"] = 1
        app = _render_with_worker(
            _CompletedWorker(results=results),
            events=events,
        )
        rendered_alerts = "\n".join(item.value for item in app.markdown)

        self.assertFalse(app.exception)
        self.assertIn("COLLISION DETECTED", rendered_alerts)
        self.assertIn("Simulation time: 8.2 s", rendered_alerts)
        self.assertIn("car_01, emergency_truck", rendered_alerts)
        self.assertIn("Acknowledge collision alert", [item.label for item in app.button])
        self.assertNotIn("NO COLLISION RECORDED", [item.value for item in app.success])
        app.button(key="acknowledge-collision").click().run()
        self.assertNotIn(
            "COLLISION DETECTED",
            "\n".join(item.value for item in app.markdown),
        )

    def test_infrastructure_paths_show_distinct_delivery_notifications(self):
        events = [
            WorkerEvent(
                "V2X_TRANSMISSION",
                5.0,
                {
                    "communication_type": "V2I",
                    "sender_id": "emergency_truck",
                    "receiver_id": "rsu_01",
                    "message_id": "hazard-1",
                },
                1,
            ),
            WorkerEvent(
                "RSU_MESSAGE_RECEIVED",
                5.0,
                {
                    "communication_type": "V2I",
                    "sender_id": "emergency_truck",
                    "receiver_id": "rsu_01",
                    "message": {"message_id": "hazard-1"},
                },
                1,
            ),
            WorkerEvent(
                "V2X_TRANSMISSION",
                5.0,
                {
                    "communication_type": "I2V",
                    "sender_id": "rsu_01",
                    "receiver_id": "car_01",
                    "message_id": "hazard-1",
                },
                1,
            ),
            WorkerEvent(
                "V2X_MESSAGE_RECEIVED",
                5.0,
                {
                    "communication_type": "I2V",
                    "vehicle_id": "car_01",
                    "message": {
                        "message_id": "hazard-1",
                        "receiver_id": "car_01",
                        "warning_distance_m": 30.0,
                        "ttc_at_warning_s": 3.0,
                        "response_recommended": False,
                        "same_lane_risk": True,
                    },
                },
                1,
            ),
        ]
        app = _render_with_worker(_CompletedWorker(), events=events)
        rendered_alerts = "\n".join(item.value for item in app.markdown)

        self.assertFalse(app.exception)
        self.assertIn("V2I MESSAGE RECEIVED", rendered_alerts)
        self.assertIn("RSU WARNING BROADCAST", rendered_alerts)
        self.assertIn("Intended recipients: 1; successful deliveries: 1", rendered_alerts)

    def test_comparison_renders_four_result_cards_and_charts(self):
        comparison_results = {}
        for index, mode in enumerate(CommunicationMode):
            comparison_results[mode.value] = {
                "metrics": {
                    "minimum_vehicle_separation_m": 12.0 + index,
                    "minimum_ttc_s": 2.0 + index,
                    "vehicles_responding": index,
                    "collisions": 0,
                }
            }
        results = {"comparison_results": comparison_results}
        app = _render_with_worker(
            _CompletedWorker(results=results, comparison=True),
            events=(),
            comparison=True,
        )
        rendered_markdown = "\n".join(item.value for item in app.markdown)

        self.assertFalse(app.exception)
        for label in ("V2X Disabled", "V2V Only", "V2I Only", "Combined V2X"):
            self.assertIn(label, rendered_markdown)
        self.assertIn("Minimum separation by mode", rendered_markdown)
        self.assertIn("Minimum TTC by mode", rendered_markdown)
        self.assertTrue(app.get("vega_lite_chart"))
        self.assertTrue(
            any("do not demonstrate a collision prevented" in item.value
                for item in app.info)
        )

    def test_real_worker_comparison_values_render_in_mode_cards(self):
        worker = SimulationWorker(
            SimulationSettings(end_time=10.0),
            comparison_modes=tuple(CommunicationMode),
        )
        worker.start()
        self.assertTrue(worker.join(timeout=60))
        self.assertEqual(worker.state, WorkerState.COMPLETED, worker.error)
        results = worker.results["comparison_results"]
        while not worker.event_queue.empty():
            worker.event_queue.get_nowait()

        app = _render_with_worker(
            worker,
            events=(),
            comparison=True,
        )
        rendered_metrics = {item.value for item in app.metric}

        self.assertFalse(app.exception)
        for scenario in results.values():
            metrics = scenario["metrics"]
            if metrics["minimum_vehicle_separation_m"] is not None:
                self.assertIn(
                    f"{metrics['minimum_vehicle_separation_m']:.2f} m",
                    rendered_metrics,
                )
            if metrics["minimum_ttc_s"] is not None:
                self.assertIn(
                    f"{metrics['minimum_ttc_s']:.2f} s",
                    rendered_metrics,
                )
            self.assertIn(str(metrics["vehicles_responding"]), rendered_metrics)
            self.assertIn(str(metrics["collisions"]), rendered_metrics)


if __name__ == "__main__":
    unittest.main()
