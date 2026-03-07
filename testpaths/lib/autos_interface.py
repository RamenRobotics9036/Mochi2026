from abc import ABC, abstractmethod

from .paths_interface import PathsInterface


class AutosInterface(ABC):
    @property
    @abstractmethod
    def name(self) -> str:
        raise NotImplementedError("Not implemented")

    @abstractmethod
    def get_path_objs(self) -> list[PathsInterface]:
        raise NotImplementedError("Not implemented")

    @abstractmethod
    def check_all_path_files_exist(self) -> bool:
        raise NotImplementedError("Not implemented")

    @abstractmethod
    def check_path_states_are_consistent(self, errors: list[str]) -> None:
        raise NotImplementedError("Not implemented")
