import json
import math
from pathlib import Path

from .app_state import AppState
from .path_state import PathState
from .paths_interface import PathsInterface


class Paths(PathsInterface):
    def __init__(self, path_file: Path, app_state: AppState) -> None:
        self._path_file = path_file
        self._app_state = app_state

    @property
    def name(self) -> str:
        return self._path_file.stem

    def exists(self) -> bool:
        return self._path_file.exists()

    def get_state(self) -> PathState:
        path_json = json.loads(self._path_file.read_text())
        return PathState(
            start_velocity=path_json["idealStartingState"]["velocity"],
            start_rotation=path_json["idealStartingState"]["rotation"],
            end_velocity=path_json["goalEndState"]["velocity"],
            end_rotation=path_json["goalEndState"]["rotation"],
        )

    def get_all_rotations(self) -> list[float]:
        path_json = json.loads(self._path_file.read_text())
        rotations = [
            path_json["idealStartingState"]["rotation"],
            path_json["goalEndState"]["rotation"],
        ]
        for target in path_json.get("rotationTargets", []):
            rotations.append(target["rotationDegrees"])
        return rotations

    def get_waypoint_segment_angles(self) -> list[float]:
        path_json = json.loads(self._path_file.read_text())
        anchors = [w["anchor"] for w in path_json.get("waypoints", [])]
        angles = []
        for a, b in zip(anchors, anchors[1:]):
            dx = b["x"] - a["x"]
            dy = b["y"] - a["y"]
            angle = math.degrees(math.atan2(dy, dx)) % 360
            angles.append(angle)
        return angles

    def get_control_point_angles(self) -> list[tuple[str, float]]:
        path_json = json.loads(self._path_file.read_text())
        results = []
        for i, w in enumerate(path_json.get("waypoints", [])):
            anchor = w["anchor"]
            if w.get("nextControl") is not None:
                c = w["nextControl"]
                angle = math.degrees(math.atan2(c["y"] - anchor["y"], c["x"] - anchor["x"])) % 360
                results.append((f"waypoint {i} nextControl", angle))
            if w.get("prevControl") is not None:
                c = w["prevControl"]
                # prevControl→anchor: direction the robot arrives from
                angle = math.degrees(math.atan2(anchor["y"] - c["y"], anchor["x"] - c["x"])) % 360
                results.append((f"waypoint {i} prevControl", angle))
        return results

    def get_start_position(self) -> tuple[float, float]:
        path_json = json.loads(self._path_file.read_text())
        anchor = path_json["waypoints"][0]["anchor"]
        return (anchor["x"], anchor["y"])

    def get_end_position(self) -> tuple[float, float]:
        path_json = json.loads(self._path_file.read_text())
        anchor = path_json["waypoints"][-1]["anchor"]
        return (anchor["x"], anchor["y"])

    def get_waypoints(self) -> list[dict]:
        path_json = json.loads(self._path_file.read_text())
        return path_json.get("waypoints", [])
