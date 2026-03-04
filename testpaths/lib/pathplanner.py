from .app_state import AppState
from .autos import Autos
from .autos_interface import AutosInterface
from .pathplanner_interface import PathPlannerInterface


class PathPlanner(PathPlannerInterface):
    def __init__(self, app_state: AppState) -> None:
        self._app_state = app_state
        self._autos = self._load_all_autos()

    def _load_all_autos(self) -> list[AutosInterface]:
        autos_dir = self._app_state.dirs.get_autos_dir()
        return [
            Autos(f, self._app_state)
            for f in sorted(autos_dir.glob("*.auto"))
        ]

    def get_auto_objs(self) -> list[AutosInterface]:
        return self._autos
