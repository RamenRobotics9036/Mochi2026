from .project_dirs import ProjectDirs


class AppState:
    def __init__(self) -> None:
        self.dirs = ProjectDirs()
