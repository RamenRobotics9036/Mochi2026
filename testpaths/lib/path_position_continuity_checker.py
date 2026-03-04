from .autos_interface import AutosInterface

_POSITION_TOLERANCE_M = 0.01  # 1 cm


class PathPositionContinuityChecker:
    @staticmethod
    def check(auto: AutosInterface, errors: list[str]) -> None:
        paths = [p for p in auto.get_path_objs() if p.exists()]
        for i in range(1, len(paths)):
            prev = paths[i - 1]
            curr = paths[i]
            end_x, end_y = prev.get_end_position()
            start_x, start_y = curr.get_start_position()
            dist = ((start_x - end_x) ** 2 + (start_y - end_y) ** 2) ** 0.5
            if dist > _POSITION_TOLERANCE_M:
                errors.append(
                    f"({end_x:.3f}, {end_y:.3f}) and '{curr.name}' start "
                    f"({start_x:.3f}, {start_y:.3f}) is {dist:.3f} m — paths must connect"
                )
