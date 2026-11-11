
import time
import math
from dataclasses import asdict, replace
from queue import Empty

import streamlit as st
import matplotlib.pyplot as plt

from browser_renderer import render_highway_canvas
from simulation_worker import SimulationWorker, WorkerState
from simulation import run_simulation
from sumo_simulation import SimulationSettings
from v2x_communication import CommunicationMode


def _initialize_command_center_state() -> SimulationWorker:
    if "command_center_worker" not in st.session_state:
        st.session_state.command_center_worker = SimulationWorker(
            SimulationSettings(),
            telemetry_step_delay_seconds=0.1,
        )
        st.session_state.command_center_telemetry = None
        st.session_state.command_center_events = []
        st.session_state.command_center_comparison = False
    st.session_state.setdefault("command_center_comparison", False)
    st.session_state.setdefault("command_center_render_generation", 0)
    return st.session_state.command_center_worker


def _warning_level(message: dict[str, object]) -> str:
    if not message.get("same_lane_risk"):
        return "OTHER LANE"
    ttc = message.get("ttc_at_warning_s")
    if ttc is None:
        return "NOT CLOSING"
    ttc_value = float(ttc)
    if ttc_value <= 3:
        return "CRITICAL"
    if ttc_value <= 6:
        return "HIGH"
    if message.get("response_recommended"):
        return "HAZARD"
    return "MONITOR"


@st.fragment(run_every=0.25)
def _render_command_center_live() -> None:
    worker = _initialize_command_center_state()
    ready = worker.state is WorkerState.READY

    mode_labels = {
        CommunicationMode.DISABLED: "V2X disabled (baseline)",
        CommunicationMode.V2V: "V2V only",
        CommunicationMode.V2I: "V2I only",
        CommunicationMode.COMBINED: "Combined V2X",
    }
    selected_mode = st.selectbox(
        "Communication mode",
        options=list(CommunicationMode),
        format_func=mode_labels.__getitem__,
        index=list(CommunicationMode).index(worker.communication_mode),
        disabled=not ready,
        key="command_center_mode",
    )
    rsu_range = st.slider(
        "RSU communication range (m)",
        min_value=50,
        max_value=500,
        value=int(worker.settings.roadside_unit.communication_range_m),
        step=25,
        disabled=not ready,
        key="command_center_rsu_range",
    )
    if ready:
        worker.communication_mode = selected_mode
        worker.settings = replace(
            worker.settings,
            roadside_unit=replace(
                worker.settings.roadside_unit,
                communication_range_m=float(rsu_range),
            ),
        )

    controls = st.columns(6)
    if controls[0].button(
        "▶ Start",
        type="primary",
        disabled=not ready,
        width="stretch",
    ):
        try:
            st.session_state.command_center_render_generation += 1
            st.session_state.command_center_telemetry = None
            st.session_state.command_center_events = []
            worker.comparison_modes = None
            st.session_state.command_center_comparison = False
            worker.start()
        except RuntimeError as error:
            st.error(str(error))
    if controls[1].button(
        "⏸ Pause",
        disabled=worker.state is not WorkerState.RUNNING,
        width="stretch",
    ):
        try:
            worker.pause()
        except RuntimeError as error:
            st.warning(str(error))
    if controls[2].button(
        "▶ Resume",
        disabled=worker.state
        not in (WorkerState.PAUSED, WorkerState.PAUSE_REQUESTED),
        width="stretch",
    ):
        try:
            worker.resume()
        except RuntimeError as error:
            st.warning(str(error))
    if controls[3].button(
        "⏹ Stop",
        disabled=worker.state
        not in (
            WorkerState.RUNNING,
            WorkerState.PAUSE_REQUESTED,
            WorkerState.PAUSED,
            WorkerState.RESUME_REQUESTED,
        ),
        width="stretch",
    ):
        worker.stop()
    if controls[4].button(
        "↺ Reset",
        width="stretch",
    ):
        try:
            worker.reset()
        except TimeoutError as error:
            st.error(str(error))
        else:
            st.session_state.command_center_telemetry = None
            st.session_state.command_center_events = []
            st.session_state.command_center_comparison = False
            st.session_state.command_center_render_generation += 1
            worker.comparison_modes = None

    if controls[5].button(
        "Compare all modes",
        help="Run disabled, V2V, V2I, and combined scenarios sequentially.",
        disabled=not ready,
        width="stretch",
    ):
        try:
            worker.comparison_modes = tuple(CommunicationMode)
            st.session_state.command_center_comparison = True
            st.session_state.command_center_render_generation += 1
            st.session_state.command_center_telemetry = None
            st.session_state.command_center_events = []
            worker.start()
        except RuntimeError as error:
            st.error(str(error))

    while True:
        try:
            st.session_state.command_center_telemetry = (
                worker.telemetry_queue.get_nowait()
            )
        except Empty:
            break

    while True:
        try:
            st.session_state.command_center_events.append(
                worker.event_queue.get_nowait()
            )
        except Empty:
            break

    telemetry = st.session_state.command_center_telemetry
    events = st.session_state.command_center_events
    state = worker.state

    st.caption(f"Simulation status: **{state.value.replace('_', ' ').title()}**")
    if worker.error:
        st.error(worker.error)

    st.subheader("Live SUMO Highway")
    render_highway_canvas(
        telemetry=asdict(telemetry) if telemetry is not None else None,
        vehicle_communication_range_m=worker.settings.communication_range_m,
        communication_links=[
            {
                **event.data,
                "simulation_time": event.simulation_time,
            }
            for event in events
            if event.event_type == "V2X_TRANSMISSION"
        ],
        key=(
            "command-center-highway-"
            f"{st.session_state.command_center_render_generation}"
        ),
    )

    vehicles = telemetry.vehicles if telemetry is not None else ()
    if vehicles:
        st.subheader("Vehicle Telemetry")
        st.dataframe(
            [
                {
                    "Vehicle ID": vehicle.vehicle_id,
                    "Type": vehicle.vehicle_type,
                    "Lane": vehicle.lane_id,
                    "Position (m)": round(vehicle.position, 2),
                    "Speed (km/h)": round(vehicle.speed * 3.6, 1),
                }
                for vehicle in vehicles
            ],
            width="stretch",
            hide_index=True,
        )

    received_messages = [
        event.data["message"]
        for event in events
        if event.event_type == "V2X_MESSAGE_RECEIVED"
    ]
    response_events = [
        event for event in events
        if event.event_type == "V2X_BRAKE_RESPONSE"
    ]
    results = worker.results
    comparison_results = (
        results.get("comparison_results")
        if results and st.session_state.command_center_comparison
        else None
    )
    metrics = results.get("metrics") if results else None
    st.subheader("V2X Performance")
    if comparison_results:
        st.caption("Completed scenario metrics (all communication values are modeled).")
        st.dataframe(
            [
                {
                    "Mode": mode_labels[CommunicationMode(mode)],
                    "Collision pairs": scenario["metrics"]["collisions"],
                    "Minimum separation (m)": (
                        scenario["metrics"]["minimum_vehicle_separation_m"]
                    ),
                    "Minimum TTC (s)": scenario["metrics"]["minimum_ttc_s"],
                    "Responding vehicles": (
                        scenario["metrics"]["vehicles_responding"]
                    ),
                    "Warning distance (m)": scenario["metrics"]["warning_distance_m"],
                    "Delivery ratio (%)": (
                        scenario["metrics"]["packet_delivery_ratio_percent"]
                    ),
                    "Modeled latency (ms)": (
                        scenario["metrics"]["communication_latency_ms"]
                    ),
                }
                for mode, scenario in comparison_results.items()
            ],
            width="stretch",
            hide_index=True,
        )
    if metrics:
        metric_values = [
            ("Broadcasts", metrics["messages_transmitted"]),
            ("Transmission attempts", metrics["transmission_attempts"]),
            ("Messages received", metrics["messages_received"]),
            (
                "Packet delivery ratio",
                f"{metrics['packet_delivery_ratio_percent']:.1f}%",
            ),
            (
                "Modeled latency",
                (
                    f"{metrics['communication_latency_ms']:.4f} ms"
                    if metrics["communication_latency_ms"] is not None
                    else "—"
                ),
            ),
            (
                "Mean warning distance",
                (
                    f"{metrics['warning_distance_m']:.1f} m"
                    if metrics["warning_distance_m"] is not None
                    else "—"
                ),
            ),
            (
                "Mean TTC at warning",
                (
                    f"{metrics['ttc_at_warning_s']:.2f} s"
                    if metrics["ttc_at_warning_s"] is not None
                    else "—"
                ),
            ),
            ("Responding vehicles", metrics["vehicles_responding"]),
            ("Collision pairs", metrics["collisions"]),
            (
                "Responders without collision",
                metrics["vehicles_avoiding_collisions"],
            ),
            (
                "Minimum same-lane separation",
                (
                    f"{metrics['minimum_vehicle_separation_m']:.2f} m"
                    if metrics["minimum_vehicle_separation_m"] is not None
                    else "—"
                ),
            ),
            (
                "Minimum positive-gap TTC",
                (
                    f"{metrics['minimum_ttc_s']:.2f} s"
                    if metrics["minimum_ttc_s"] is not None
                    else "—"
                ),
            ),
        ]
    else:
        warning_distances = [
            float(message["warning_distance_m"])
            for message in received_messages
        ]
        warning_ttcs = [
            float(message["ttc_at_warning_s"])
            for message in received_messages
            if message["ttc_at_warning_s"] is not None
        ]
        modeled_latencies = [
            float(message["communication_latency_ms"])
            for message in received_messages
        ]
        active = state in (
            WorkerState.RUNNING,
            WorkerState.PAUSE_REQUESTED,
            WorkerState.PAUSED,
            WorkerState.RESUME_REQUESTED,
        )
        message_count = len(received_messages)
        metric_values = [
            (
                "Broadcasts",
                sum(
                    event.event_type == "EMERGENCY_BRAKING"
                    for event in events
                ),
            ),
            ("Transmission attempts", message_count),
            ("Messages received", message_count),
            (
                "Packet delivery ratio",
                "Pending"
                if not message_count
                else "100% modeled",
            ),
            (
                "Modeled latency",
                (
                    f"{sum(modeled_latencies) / len(modeled_latencies):.4f} ms"
                    if modeled_latencies
                    else "—"
                ),
            ),
            (
                "Mean warning distance",
                (
                    f"{sum(warning_distances) / len(warning_distances):.1f} m"
                    if warning_distances
                    else "—"
                ),
            ),
            (
                "Mean TTC at warning",
                (
                    f"{sum(warning_ttcs) / len(warning_ttcs):.2f} s"
                    if warning_ttcs
                    else "—"
                ),
            ),
            ("Responding vehicles", len(response_events)),
            ("Collision pairs", "Pending" if active else "Unavailable"),
            ("Responders without collision", "Pending" if active else "—"),
            ("Minimum same-lane separation", "Pending" if active else "Unavailable"),
            ("Minimum positive-gap TTC", "Pending" if active else "Unavailable"),
        ]

    for row_start in range(0, len(metric_values), 5):
        metric_columns = st.columns(5)
        for column, (label, value) in zip(
            metric_columns,
            metric_values[row_start:row_start + 5],
        ):
            column.metric(label, value)
    st.caption(
        "Communication metrics are in-process simulation values, not wireless "
        "measurements. Warning distance/TTC are values at message reception. "
        "Responders without collision is not a counterfactual prevention claim."
    )

    if received_messages:
        st.subheader("Receiver Risk Assessments")
        st.dataframe(
            [
                {
                    "Vehicle": message["receiver_id"],
                    "Warning level": _warning_level(message),
                    "Warning distance (m)": message["warning_distance_m"],
                    "TTC at reception (s)": message["ttc_at_warning_s"],
                    "Brake response": (
                        "Scheduled"
                        if message["response_recommended"]
                        else "Not recommended"
                    ),
                }
                for message in received_messages
            ],
            width="stretch",
            hide_index=True,
        )

    st.subheader("V2X Event Log")
    simulation_events = [
        event for event in events
        if event.event_type not in ("WORKER_STATE",)
    ]
    if simulation_events:
        st.dataframe(
            [
                {
                    "Simulation time (s)": event.simulation_time,
                    "Event": event.event_type,
                    "Sender": (
                        event.data.get("sender_id")
                        or event.data.get("message", {}).get("sender_id", "")
                    ),
                    "Receiver": (
                        event.data.get("receiver_id")
                        or event.data.get("vehicle_id", "")
                        or event.data.get("message", {}).get("receiver_id", "")
                    ),
                    "Communication": (
                        event.data.get("communication_type")
                        or event.data.get("message", {}).get(
                            "communication_type", ""
                        )
                    ),
                    "Details": (
                        event.data.get("error")
                        or event.data.get("risk_reason")
                        or event.data.get("message_id")
                        or ""
                    ),
                }
                for event in reversed(simulation_events)
            ],
            width="stretch",
            hide_index=True,
        )
    else:
        st.caption("No V2X events yet.")


def _render_command_center() -> None:
    st.title("V2X Command Center")
    st.caption(
        "SUMO traffic, vehicle telemetry, V2X risk assessments, and "
        "communication metrics in one browser view."
    )
    _render_command_center_live()


# --------------------------------------------------
# PAGE CONFIGURATION
# --------------------------------------------------

st.set_page_config(
    page_title="V2X Safety Monitor",
    page_icon="🚗",
    layout="wide"
)

application_view = st.sidebar.radio(
    "Application view",
    ("Legacy Dashboard", "V2X Command Center"),
    key="application_view",
)

if application_view == "V2X Command Center":
    _render_command_center()
    st.stop()

st.title("🚘 V2X Collision Avoidance System")
st.caption(
    "CONNECTED VEHICLE SAFETY MONITOR | "
    "Emergency Braking & Collision Detection"
)

st.divider()


# --------------------------------------------------
# SIDEBAR CONTROLS
# --------------------------------------------------

st.sidebar.title("Simulation Controls")

playback_speed = st.sidebar.select_slider(
    "Playback speed",
    options=["Slow", "Normal", "Fast"],
    value="Normal"
)

delay_settings = {
    "Slow": 0.15,
    "Normal": 0.07,
    "Fast": 0.02
}

st.sidebar.caption(
    "Adjust how quickly the simulation plays."
)

run_button = st.sidebar.button(
    "▶ Start Simulation",
    type="primary",
    use_container_width=True
)

st.sidebar.divider()

st.sidebar.markdown("### Scenario")
st.sidebar.write("Vehicle A: Emergency braking")
st.sidebar.write("Vehicle B: Approaching car")
st.sidebar.write("Initial separation: 100 m")


# --------------------------------------------------
# DASHBOARD PLACEHOLDERS
# --------------------------------------------------

overview_area = st.empty()
warning_area = st.empty()
vehicle_area = st.empty()
road_area = st.empty()
progress_area = st.empty()


# --------------------------------------------------
# SIMULATION EXECUTION
# --------------------------------------------------

if run_button:

    # Generate simulation data
    results = run_simulation()

    history = results["history"]
    message_log = results["message_log"]

    st.toast("Simulation started!", icon="🚗")

    # Playback every recorded simulation step
    for index, frame in enumerate(history):

        time_value = frame["time"]
        separation = frame["separation"]
        ttc = frame["ttc"]
        warning = frame["warning_level"]

        # ------------------------------------------
        # OVERVIEW METRICS
        # ------------------------------------------

        with overview_area.container():

            st.subheader("Live Simulation Overview")

            col1, col2, col3, col4 = st.columns(4)

            col1.metric(
                "Simulation Time",
                f"{time_value:.1f} s"
            )

            col2.metric(
                "Vehicle Separation",
                f"{max(0, separation):.2f} m"
            )

            col3.metric(
                "Truck Speed",
                f"{frame['truck_speed'] * 3.6:.1f} km/h"
            )

            col4.metric(
                "Car Speed",
                f"{frame['car_speed'] * 3.6:.1f} km/h"
            )

        # ------------------------------------------
        # WARNING SYSTEM
        # ------------------------------------------

        with warning_area.container():

            st.subheader("Collision Warning System")

            if warning == "CRITICAL":
                st.error("🔴 CRITICAL: Collision risk is imminent!")

            elif warning == "HIGH":
                st.warning("🟠 HIGH RISK: Immediate driver attention required.")

            elif warning == "HAZARD DETECTED":
                st.info("🟡 HAZARD DETECTED: Emergency braking notification received.")

            else:
                st.success("🟢 NO WARNING: No active hazard notification.")

            if ttc is not None:
                st.write(f"Estimated Time-to-Collision: **{ttc:.2f} seconds**")
            else:
                st.write("Estimated Time-to-Collision: Not closing")

        # ------------------------------------------
        # VEHICLE INFORMATION
        # ------------------------------------------

        with vehicle_area.container():

            st.subheader("Vehicle Telemetry")

            col1, col2 = st.columns(2)

            with col1:
                st.markdown("### 🚛 Vehicle A — Truck")
                st.write(f"Position: {frame['truck_position']:.2f} m")
                st.write(
                    f"Speed: {frame['truck_speed'] * 3.6:.1f} km/h"
                )

            with col2:
                st.markdown("### 🚗 Vehicle B — Car")
                st.write(f"Position: {frame['car_position']:.2f} m")
                st.write(
                    f"Speed: {frame['car_speed'] * 3.6:.1f} km/h"
                )

        # ------------------------------------------
        # ANIMATED ROAD
        # ------------------------------------------

        with road_area.container():

            st.subheader("Live Vehicle Tracking")

            fig, ax = plt.subplots(figsize=(11, 2.6))

            # Road boundaries
            ax.axhline(0, color="gray", linewidth=3)
            ax.axhline(-0.35, color="gray", linewidth=1)

            # Road markings
            for marker in range(
                math.floor(frame["car_position"] / 10) * 10,
                int(frame["truck_position"]) + 30,
                10
            ):
                ax.plot(
                    [marker, marker + 5],
                    [-0.18, -0.18],
                    color="gray",
                    linewidth=2
                )

            # Vehicle positions
            ax.scatter(
                frame["truck_position"], 0,
                s=450, marker="s",
                label="Truck"
            )

            ax.scatter(
                frame["car_position"], 0,
                s=250, marker="o",
                label="Car"
            )

            # Vehicle labels
            ax.annotate(
                "TRUCK",
                (frame["truck_position"], 0.12),
                ha="center"
            )

            ax.annotate(
                "CAR",
                (frame["car_position"], 0.12),
                ha="center"
            )

            # Collision marker
            if separation <= 0:
                collision_position = (
                    frame["truck_position"] +
                    frame["car_position"]
                ) / 2

                ax.axvline(
                    collision_position,
                    color="red",
                    linestyle="--",
                    label="Collision point"
                )

            # Keep the road centred around the vehicles
            left_limit = min(
                frame["car_position"],
                frame["truck_position"]
            ) - 20

            right_limit = max(
                frame["car_position"],
                frame["truck_position"]
            ) + 30

            ax.set_xlim(left_limit, right_limit)
            ax.set_ylim(-0.55, 0.65)
            ax.set_yticks([])
            ax.set_xlabel("Road position (metres)")
            ax.legend(loc="upper right")
            ax.grid(axis="x", alpha=0.2)

            st.pyplot(fig)
            plt.close(fig)

        # ------------------------------------------
        # PLAYBACK PROGRESS
        # ------------------------------------------

        progress_area.progress(
            (index + 1) / len(history),
            text=f"Simulation progress: {time_value:.1f} seconds"
        )

        time.sleep(delay_settings[playback_speed])

    # --------------------------------------------------
    # FINAL RESULTS
    # --------------------------------------------------

    final_state = history[-1]

    st.divider()
    st.header("Simulation Results")

    col1, col2, col3 = st.columns(3)

    col1.metric(
        "Messages Received",
        len(message_log)
    )

    col2.metric(
        "Simulation Duration",
        f"{final_state['time']:.1f} s"
    )

    col3.metric(
        "Collision Status",
        "Detected" if results["collision_detected"]
        else "Avoided"
    )

    if results["collision_detected"]:
        st.error(
            f"Collision occurred at "
            f"{results['collision_time']:.1f} seconds."
        )
    else:
        st.success("No collision occurred during the simulation.")

    # --------------------------------------------------
    # PERFORMANCE GRAPHS
    # --------------------------------------------------

    st.subheader("Vehicle Separation Analysis")

    times = [item["time"] for item in history]
    distances = [
        item["separation"] for item in history
    ]

    fig, ax = plt.subplots(figsize=(10, 4))

    ax.plot(times, distances, label="Vehicle Separation")
    ax.axhline(
        0, linestyle="--",
        label="Collision Point"
    )

    ax.set_xlabel("Time (seconds)")
    ax.set_ylabel("Separation (metres)")
    ax.legend()
    ax.grid(alpha=0.3)

    st.pyplot(fig)
    plt.close(fig)

    st.subheader("Time-to-Collision Analysis")

    ttc_values = [
        item["ttc"] if item["ttc"] is not None
        else float("nan")
        for item in history
    ]

    fig, ax = plt.subplots(figsize=(10, 4))

    ax.plot(times, ttc_values, label="Estimated TTC")
    ax.axhline(3, linestyle="--", label="Critical Threshold")
    ax.axhline(6, linestyle=":", label="High Warning Threshold")

    ax.set_xlabel("Time (seconds)")
    ax.set_ylabel("TTC (seconds)")
    ax.legend()
    ax.grid(alpha=0.3)

    st.pyplot(fig)
    plt.close(fig)

    # --------------------------------------------------
    # V2X MESSAGE LOG
    # --------------------------------------------------

    st.subheader("V2X Communication Log")

    if message_log:
        for message in message_log:
            st.json(message)
    else:
        st.warning("No V2X messages received.")

    st.success("Simulation playback completed!")
