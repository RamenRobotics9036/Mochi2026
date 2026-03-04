from typing import Optional

from .autos_interface import AutosInterface

_CARDINAL_ANGLES = [0.0, 90.0, 180.0, 270.0]
_NEAR_THRESHOLD_DEG = 5.0


class WaypointsNotAskewChecker:
    @staticmethod
    def _nearest_cardinal(degrees: float) -> Optional[float]:
        """Returns the cardinal angle that `degrees` is within _NEAR_THRESHOLD_DEG of, or None."""
        for cardinal in _CARDINAL_ANGLES:
            if abs(degrees - cardinal) <= _NEAR_THRESHOLD_DEG:
                return cardinal
        return None

    @staticmethod
    def check(auto: AutosInterface, errors: list[str]) -> None:
        for path in auto.get_path_objs():
            if not path.exists():
                continue
            for i, angle in enumerate(path.get_waypoint_segment_angles()):
                cardinal = WaypointsNotAskewChecker._nearest_cardinal(angle)
                if cardinal is not None and abs(angle - cardinal) > 1e-9:
                    errors.append(
                        f"{auto.name}: path '{path.name}' segment {i}→{i+1} has angle "
                        f"{angle:.2f}° which is near {cardinal:.0f}° but not exactly aligned — "
                        f"set it to exactly {cardinal:.0f}°"
                    )
            for label, angle in path.get_control_point_angles():
                cardinal = WaypointsNotAskewChecker._nearest_cardinal(angle)
                if cardinal is not None and abs(angle - cardinal) > 1e-9:
                    errors.append(
                        f"{auto.name}: path '{path.name}' {label} has angle "
                        f"{angle:.2f}° which is near {cardinal:.0f}° but not exactly aligned — "
                        f"set it to exactly {cardinal:.0f}°"
                    )
