class PathState:
    def __init__(self, start_velocity: float, start_rotation: float,
                 end_velocity: float, end_rotation: float) -> None:
        self.start_velocity = start_velocity
        self.start_rotation = start_rotation
        self.end_velocity = end_velocity
        self.end_rotation = end_rotation
