from abc import ABC, abstractmethod

from .autos_interface import AutosInterface


class PathPlannerInterface(ABC):
    @abstractmethod
    def get_auto_objs(self) -> list[AutosInterface]:
        raise NotImplementedError("Not implemented")
