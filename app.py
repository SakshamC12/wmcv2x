
import time
import math
import streamlit as st
import matplotlib.pyplot as plt

from simulation import run_simulation


# --------------------------------------------------
# PAGE CONFIGURATION
# --------------------------------------------------

st.set_page_config(
    page_title="V2X Safety Monitor",
    page_icon="🚗",
    layout="wide"
)

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
