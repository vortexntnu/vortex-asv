import dataclasses


@dataclasses.dataclass(frozen=True)
class Waypoint2D:
    north: float
    east: float