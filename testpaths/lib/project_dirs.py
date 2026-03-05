from pathlib import Path


class ProjectDirs:
    def __init__(self) -> None:
        self._root = self._find_project_root()

    def _find_project_root(self) -> Path:
        directory = Path(__file__).resolve().parent
        while directory != directory.parent:
            if (directory / "build.gradle").exists():
                return directory
            directory = directory.parent
        raise RuntimeError("Could not locate project root (no build.gradle found)")

    def get_autos_dir(self) -> Path:
        return self._root / "src" / "main" / "deploy" / "pathplanner" / "autos"

    def get_paths_dir(self) -> Path:
        return self._root / "src" / "main" / "deploy" / "pathplanner" / "paths"
