from .autos_interface import AutosInterface
from .utils import approx_equal


class PathStateChecker:
    @staticmethod
    def check(auto: AutosInterface, errors: list[str]) -> None:
        paths = auto.get_path_objs()

        if not paths:
            errors.append(f"{auto.name}: must reference at least one path")
            return

        if not auto.check_all_path_files_exist():
            return

        states = [p.get_state() for p in paths]

        # a) First path must start at velocity 0
        if not approx_equal(states[0].start_velocity, 0.0):
            errors.append(
                f"{auto.name}: first path '{paths[0].name}' "
                f"idealStartingState.velocity must be 0, got {states[0].start_velocity}"
            )

        # b) Last path must end at velocity 0
        if not approx_equal(states[-1].end_velocity, 0.0):
            errors.append(
                f"{auto.name}: last path '{paths[-1].name}' "
                f"goalEndState.velocity must be 0, got {states[-1].end_velocity}"
            )

        # c) Each path's idealStartingState must match the previous path's goalEndState
        for i in range(1, len(paths)):
            prev, curr = states[i - 1], states[i]
            if not approx_equal(curr.start_velocity, prev.end_velocity):
                errors.append(
                    f"{auto.name}: '{paths[i].name}' idealStartingState.velocity "
                    f"({curr.start_velocity}) != '{paths[i-1].name}' goalEndState.velocity "
                    f"({prev.end_velocity})"
                )
            if not approx_equal(curr.start_rotation, prev.end_rotation):
                errors.append(
                    f"{auto.name}: '{paths[i].name}' idealStartingState.rotation "
                    f"({curr.start_rotation}) != '{paths[i-1].name}' goalEndState.rotation "
                    f"({prev.end_rotation})"
                )
