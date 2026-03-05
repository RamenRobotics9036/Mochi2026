import json
from pathlib import Path

from .app_state import AppState
from .autos_interface import AutosInterface
from .json_parser import JsonParser
from .path_state_checker import PathStateChecker
from .paths import Paths
from .paths_interface import PathsInterface


class Autos(AutosInterface):
    def __init__(self, auto_file: Path, app_state: AppState) -> None:
        self._auto_file = auto_file
        self._app_state = app_state
        self._json = json.loads(auto_file.read_text())
        self._parser = JsonParser(app_state.dirs.get_paths_dir())
        self._paths = self._load_paths()

    def _load_paths(self) -> list[PathsInterface]:
        paths_dir = self._app_state.dirs.get_paths_dir()
        path_names = self._parser.collect_path_names(self._json.get("command", {}))
        return [
            Paths(paths_dir / f"{name}.path", self._app_state)
            for name in path_names
        ]

    @property
    def name(self) -> str:
        return self._auto_file.name

    def get_path_objs(self) -> list[PathsInterface]:
        return self._paths

    def check_all_path_files_exist(self) -> bool:
        return all(p.exists() for p in self.get_path_objs())

    def check_path_states_are_consistent(self, errors: list[str]) -> None:
        PathStateChecker.check(self, errors)
