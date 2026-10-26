#!/usr/bin/env python3
"""Protocol tests that do not require ROS2 to be installed on the build PC."""

import ast
import math
from pathlib import Path


SOURCE = (Path(__file__).resolve().parents[2] / "ros2_ws" / "src" /
          "my_radar_follower" / "my_radar_follower" /
          "txt_tracker_bridge.py")


def load_parser():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    selected = []
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = {target.id for target in node.targets
                     if isinstance(target, ast.Name)}
            if "PREDICTED_MAX_LOST_FRAMES" in names:
                selected.append(node)
        elif (isinstance(node, ast.FunctionDef) and
              node.name == "parse_tracking_record"):
            selected.append(node)
    namespace = {"math": math}
    exec(compile(ast.Module(body=selected, type_ignores=[]),
                 str(SOURCE), "exec"), namespace)
    return namespace["parse_tracking_record"]


def record_line(state="TRACKING", measured=1, lost=0, scale=1.0,
                allow_translation=1, allow_yaw=1):
    fields = [
        "1", "0.2", "3.0", "3.0", "3.8",
        "0.2", "3.0", "3.0", "3.8", str(measured),
        "0.9", "42", str(lost), "8", "2026-07-16T10:00:00.000",
        "realtime", state, str(scale), str(allow_translation),
        str(allow_yaw), "0",
    ]
    return " ".join(fields)


def require_rejected(parser, line):
    try:
        parser(line)
    except ValueError:
        return
    raise AssertionError(f"invalid record was accepted: {line}")


def main():
    parser = load_parser()
    tracking = parser(record_line())
    assert tracking["control_state"] == "TRACKING"
    assert tracking["frame"] == 42

    # Pending reacquisition can carry a candidate measurement but must still
    # be controlled as PREDICTED according to the explicit state token.
    predicted = parser(record_line("PREDICTED", measured=1, lost=12,
                                   scale=0.4))
    assert predicted["control_state"] == "PREDICTED"
    assert predicted["speed_scale"] == 0.4

    stopped = parser(record_line("STOPPED", measured=1, lost=31, scale=0.0,
                                 allow_translation=0, allow_yaw=0))
    assert stopped["control_state"] == "STOPPED"
    assert not stopped["allow_translation"] and not stopped["allow_yaw"]

    require_rejected(parser, record_line("TRACKING", measured=0, lost=0))
    require_rejected(parser, record_line("PREDICTED", measured=0, lost=31))
    require_rejected(parser, record_line("UNKNOWN", measured=0, lost=1))
    require_rejected(parser, record_line(allow_translation=2))
    require_rejected(parser, "1 2 3")
    print("PASS: ROS 21-column tracking protocol")


if __name__ == "__main__":
    main()
