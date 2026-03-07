import math

from .autos_interface import AutosInterface

# Control handle length should be at least this fraction of the segment length.
# Below this ratio the curvature becomes too sharp to follow accurately.
_MIN_CONTROL_RATIO = 0.2


def _dist(a: dict, b: dict) -> float:
    return math.hypot(b["x"] - a["x"], b["y"] - a["y"])


class ControlPointLengthChecker:
    @staticmethod
    def check(auto: AutosInterface, errors: list[str]) -> None:
        for path in auto.get_path_objs():
            if not path.exists():
                continue
            waypoints = path.get_waypoints()
            for i, w in enumerate(waypoints):
                anchor = w["anchor"]

                # Segment length to next anchor (for nextControl ratio)
                if w.get("nextControl") is not None and i + 1 < len(waypoints):
                    segment_len = _dist(anchor, waypoints[i + 1]["anchor"])
                    handle_len = _dist(anchor, w["nextControl"])
                    if segment_len > 0:
                        ratio = handle_len / segment_len
                        if ratio < _MIN_CONTROL_RATIO:
                            errors.append(
                                f"{auto.name}: path '{path.name}' waypoint {i} nextControl "
                                f"handle is too short (ratio {ratio:.2f} < {_MIN_CONTROL_RATIO}) — "
                                f"curve will be too sharp to follow accurately"
                            )

                # Segment length from previous anchor (for prevControl ratio)
                if w.get("prevControl") is not None and i - 1 >= 0:
                    segment_len = _dist(waypoints[i - 1]["anchor"], anchor)
                    handle_len = _dist(w["prevControl"], anchor)
                    if segment_len > 0:
                        ratio = handle_len / segment_len
                        if ratio < _MIN_CONTROL_RATIO:
                            errors.append(
                                f"{auto.name}: path '{path.name}' waypoint {i} prevControl "
                                f"handle is too short (ratio {ratio:.2f} < {_MIN_CONTROL_RATIO}) — "
                                f"curve will be too sharp to follow accurately"
                            )
