from pathlib import Path

import streamlit.components.v1 as components


_render_component = components.declare_component(
    "v2x_highway_canvas",
    path=str(Path(__file__).parent / "v2x_canvas"),
)


def render_highway_canvas(
    *,
    telemetry: dict[str, object] | None,
    vehicle_communication_range_m: float,
    communication_links: list[dict[str, object]],
    key: str,
) -> None:
    _render_component(
        telemetry=telemetry,
        vehicle_communication_range_m=vehicle_communication_range_m,
        communication_links=communication_links,
        key=key,
        default=None,
        height=460,
    )
