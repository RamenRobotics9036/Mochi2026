import json
from pathlib import Path

from .path_state import PathState


class JsonParser:
    def __init__(self, paths_dir: Path) -> None:
        self._paths_dir = paths_dir

    def collect_path_names(self, node) -> list[str]:
        """Recursively collect ordered PathPlanner path names from a command tree."""
        if isinstance(node, list):
            names: list[str] = []
            for child in node:
                names.extend(self.collect_path_names(child))
            return names
        if isinstance(node, dict):
            if node.get("type") == "path":
                path_name = node.get("data", {}).get("pathName", "")
                if path_name:
                    return [path_name]
            names = []
            for value in node.values():
                names.extend(self.collect_path_names(value))
            return names
        return []

    def load_path_state(self, path_name: str) -> PathState:
        path_json = json.loads((self._paths_dir / f"{path_name}.path").read_text())
        return PathState(
            start_velocity=path_json["idealStartingState"]["velocity"],
            start_rotation=path_json["idealStartingState"]["rotation"],
            end_velocity=path_json["goalEndState"]["velocity"],
            end_rotation=path_json["goalEndState"]["rotation"],
        )
