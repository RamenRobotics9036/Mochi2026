from abc import ABC, abstractmethod

from .path_state import PathState


class PathsInterface(ABC):
    @property
    @abstractmethod
    def name(self) -> str:
        raise NotImplementedError("Not implemented")

    @abstractmethod
    def exists(self) -> bool:
        raise NotImplementedError("Not implemented")

    @abstractmethod
    def get_state(self) -> PathState:
        raise NotImplementedError("Not implemented")

    @abstractmethod
    def get_all_rotations(self) -> list[float]:
        """Returns all rotation degree values in this path: start, end, and any rotation targets."""
        raise NotImplementedError("Not implemented")

    @abstractmethod
    def get_waypoint_segment_angles(self) -> list[float]:
        """Returns the angle in degrees [0, 360) of each anchor-to-anchor segment."""
        raise NotImplementedError("Not implemented")

    @abstractmethod
    def get_control_point_angles(self) -> list[tuple[str, float]]:
        """Returns (label, angle) for each non-null control handle (anchor→nextControl or prevControl→anchor)."""
        raise NotImplementedError("Not implemented")

    @abstractmethod
    def get_start_position(self) -> tuple[float, float]:
        """Returns (x, y) of the first waypoint anchor."""
        raise NotImplementedError("Not implemented")

    @abstractmethod
    def get_end_position(self) -> tuple[float, float]:
        """Returns (x, y) of the last waypoint anchor."""
        raise NotImplementedError("Not implemented")

    @abstractmethod
    def get_waypoints(self) -> list[dict]:
        """Returns the raw waypoint list from the path JSON."""
        raise NotImplementedError("Not implemented")
