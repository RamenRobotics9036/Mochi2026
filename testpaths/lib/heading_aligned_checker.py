from typing import Optional

from .autos_interface import AutosInterface
from .utils import approx_equal

_CARDINAL_ANGLES = [0.0, 90.0, 180.0, 270.0]
_NEAR_THRESHOLD_DEG = 5.0


class HeadingAlignedChecker:
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
            for rotation in path.get_all_rotations():
                cardinal = HeadingAlignedChecker._nearest_cardinal(rotation)

                # print(f"rotation: {rotation}, cardinal: {cardinal}")

                if cardinal is not None and not approx_equal(rotation, cardinal):
                    errors.append(
                        f"{auto.name}: path '{path.name}' has heading {rotation}° which is "
                        f"near {cardinal:.0f}° but not exactly aligned — "
                        f"set it to exactly {cardinal:.0f}°"
                    )
