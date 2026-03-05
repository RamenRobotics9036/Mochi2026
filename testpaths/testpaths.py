"""
PathPlanner auto/path consistency checks.

Run with:  python testpaths/test_path_references.py
Exits 0 on success, 1 if any checks fail (errors printed to stderr).
"""

import sys

from lib.app_state import AppState
from lib.control_point_length_checker import ControlPointLengthChecker
from lib.heading_aligned_checker import HeadingAlignedChecker
from lib.path_position_continuity_checker import PathPositionContinuityChecker
from lib.pathplanner import PathPlanner
from lib.waypoints_not_askew_checker import WaypointsNotAskewChecker

_APP_STATE = AppState()
_PP = PathPlanner(_APP_STATE)


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def check_every_auto_references_only_existing_path_files(errors: list[str]) -> None:
    for auto in _PP.get_auto_objs():
        for path in auto.get_path_objs():
            if not path.exists():
                errors.append(f"{auto.name}: references missing path '{path.name}.path'")


def check_all_autos_path_states_are_consistent(errors: list[str]) -> None:
    for auto in _PP.get_auto_objs():
        auto.check_path_states_are_consistent(errors)


def check_all_headings_are_aligned(errors: list[str]) -> None:
    for auto in _PP.get_auto_objs():
        HeadingAlignedChecker.check(auto, errors)


def check_all_waypoints_not_askew(errors: list[str]) -> None:
    for auto in _PP.get_auto_objs():
        WaypointsNotAskewChecker.check(auto, errors)


def check_all_path_position_continuity(errors: list[str]) -> None:
    for auto in _PP.get_auto_objs():
        PathPositionContinuityChecker.check(auto, errors)


def check_all_control_point_lengths(errors: list[str]) -> None:
    for auto in _PP.get_auto_objs():
        ControlPointLengthChecker.check(auto, errors)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    checks = [
        ("Missing path files",           check_every_auto_references_only_existing_path_files),
        ("Path state consistency",       check_all_autos_path_states_are_consistent),
        ("Heading alignment",            check_all_headings_are_aligned),
        ("Waypoints not askew",          check_all_waypoints_not_askew),
        ("Path position continuity",     check_all_path_position_continuity),
        ("Control point lengths",        check_all_control_point_lengths),
    ]

    # $TODO - We should check for consistency in the global constants in paths,
    # like max rotation velocity, etc.

    # $TODO - We should check that resetodometry is set for all paths!

    any_errors = False
    for header, check_fn in checks:
        section_errors: list[str] = []
        check_fn(section_errors)
        if section_errors:
            any_errors = True
            print(f"\n--- {header} ---", file=sys.stderr)
            for error in section_errors:
                print(f"ERROR: {error}", file=sys.stderr)

    if any_errors:
        sys.exit(1)

    print("All path checks passed.")
    sys.exit(0)


if __name__ == "__main__":
    main()
