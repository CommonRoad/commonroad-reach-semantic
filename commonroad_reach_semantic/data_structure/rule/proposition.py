import enum

from commonroad_reach_semantic.data_structure.config.outgoing_direction import OutgoingDirection


@enum.unique
class PropositionGroup(enum.Enum):
    POSITION = 0
    VELOCITY = 1
    ACCELERATION = 2
    VEHICLE = 3
    TRAFFIC_SIGN = 4
    TRAFFIC_LIGHT = 5
    INTERSECTION = 6
    PRIORITY = 7
    TRAFFIC_STATUS = 8
    TEMPORARY = 9


class Proposition:
    @staticmethod
    def in_lanelet(id_lanelet: int = None) -> str:
        prefix = 'InLanelet_'
        if not id_lanelet:
            return prefix

        else:
            return prefix + str(id_lanelet)

    @staticmethod
    def lanelet_transition(id_lanelet_source: int, id_lanelet_target: int) -> str:
        return f"Lanelet_{id_lanelet_source}_to_{id_lanelet_target}"

    @staticmethod
    def right_most_in_lanelet() -> str:
        return 'RightMostInLanelet'

    @staticmethod
    def in_same_lane(id_obstacle: int = None) -> str:
        return f"InSameLane_V{id_obstacle}"

    @staticmethod
    def behind(id_obstacle: int = None) -> str:
        prefix = 'Behind_V'
        if not id_obstacle:
            return prefix

        else:
            return prefix + str(id_obstacle)

    @staticmethod
    def beside(id_obstacle: int = None) -> str:
        prefix = 'Beside_V'
        if not id_obstacle:
            return prefix

        else:
            return prefix + str(id_obstacle)

    @staticmethod
    def in_front_of(id_obstacle: int = None) -> str:
        prefix = 'InFrontOf_V'
        if not id_obstacle:
            return prefix

        else:
            return prefix + str(id_obstacle)

    @staticmethod
    def left_of(id_obstacle: int = None) -> str:
        prefix = 'LeftOf_V'
        if not id_obstacle:
            return prefix

        else:
            return prefix + str(id_obstacle)

    @staticmethod
    def aligned_with(id_obstacle: int = None) -> str:
        prefix = 'AlignedWith_V'
        if not id_obstacle:
            return prefix

        else:
            return prefix + str(id_obstacle)

    @staticmethod
    def right_of(id_obstacle: int = None) -> str:
        prefix = 'RightOf_V'
        if not id_obstacle:
            return prefix

        else:
            return prefix + str(id_obstacle)

    @staticmethod
    def below_physical_velocity_limit() -> str:
        return 'BelowPhysicalVelocity'

    @staticmethod
    def below_lanelet_velocity_limit() -> str:
        return 'BelowLaneletVelocity'

    @staticmethod
    def above_required_velocity_limit() -> str:
        return 'AboveRequiredVelocity'

    @staticmethod
    def driving_forward() -> str:
        return 'DrivingForward'

    @staticmethod
    def safe_following_distance_to(id_obstacle: int = None) -> str:
        return 'SafeFollowingDistanceTo_' + str(id_obstacle)

    @staticmethod
    def safe_leading_distance_to(id_obstacle: int = None) -> str:
        return f'SafeLeadingDistanceTo_V{str(id_obstacle)}'

    @staticmethod
    def same_driving_direction() -> str:
        return 'SameDrivingDirection'

    @staticmethod
    def causes_braking_for(id_obstacle: int = None):
        return f"CausesBrakingFor_V{str(id_obstacle)}"

    # intersection-related
    @staticmethod
    def in_intersection(id_obstacle: int = None) -> str:
        suffix = 'InIntersection'
        if not id_obstacle:
            return suffix

        else:
            return f"V{str(id_obstacle)}_{suffix}"

    @staticmethod
    def in_left_out(id_obstacle: int = None) -> str:
        suffix = 'InLeftOut'
        if not id_obstacle:
            return suffix

        else:
            return f"V{str(id_obstacle)}_{suffix}"

    @staticmethod
    def in_straight_out(id_obstacle: int = None) -> str:
        suffix = 'InStraightOut'
        if not id_obstacle:
            return suffix

        else:
            return f"V{str(id_obstacle)}_{suffix}"

    @staticmethod
    def in_right_out(id_obstacle: int = None) -> str:
        suffix = 'InRightOut'
        if not id_obstacle:
            return suffix

        else:
            return f"V{str(id_obstacle)}_{suffix}"

    @staticmethod
    def in_direction_successor(direction: OutgoingDirection, id_obstacle: int = None) -> str:
        match direction:
            case OutgoingDirection.LEFT:
                return Proposition.in_left_successor(id_obstacle)
            case OutgoingDirection.STRAIGHT:
                return Proposition.in_straight_successor(id_obstacle)
            case OutgoingDirection.RIGHT:
                return Proposition.in_right_successor(id_obstacle)
            case _:
                raise ValueError(f"Invalid direction: {direction}")

    @staticmethod
    def in_left_successor(id_obstacle: int = None) -> str:
        suffix = 'InLeftSuc'
        if not id_obstacle:
            return suffix

        else:
            return f"V{str(id_obstacle)}_{suffix}"

    @staticmethod
    def in_straight_successor(id_obstacle: int = None) -> str:
        suffix = 'InStraightSuc'
        if not id_obstacle:
            return suffix

        else:
            return f"V{str(id_obstacle)}_{suffix}"

    @staticmethod
    def in_right_successor(id_obstacle: int = None) -> str:
        suffix = 'InRightSuc'
        if not id_obstacle:
            return suffix

        else:
            return f"V{str(id_obstacle)}_{suffix}"

    @staticmethod
    def intersection_left_of(id_obstacle: int = None) -> str:
        prefix = "Intersection_LeftOf_V"
        if not id_obstacle:
            return prefix

        else:
            return f"{prefix}{str(id_obstacle)}"

    @staticmethod
    def on_oncoming(id_obstacle: int = None) -> str:
        return f"V{id_obstacle}_OnOncoming"

    @staticmethod
    def on_oncoming_of(id_obstacle: int = None) -> str:
        return f"OnOncomingOf_V{id_obstacle}"

    @staticmethod
    def region_out_same_as_vehicle_out(region_dir: OutgoingDirection, vehicle_dir: OutgoingDirection,
                                       id_obstacle: int = None) -> str:
        return f"{region_dir}Out_SameAs_V{id_obstacle}_{vehicle_dir}Out"

    @staticmethod
    def in_conflict_with(id_obstacle: int = None) -> str:
        """Conflict caused by the reachable set."""
        return f"InConflictWith_V{id_obstacle}"

    @staticmethod
    def in_conflict_by(id_obstacle: int = None) -> str:
        """Conflict caused by the vehicle."""
        return f"InConflictBy_V{id_obstacle}"

    @staticmethod
    def has_priority(id_obstacle: int, direction_ego: OutgoingDirection, direction_other: OutgoingDirection) -> str:
        return f"Has_{direction_ego}_{direction_other}_PriorityOver_V{id_obstacle}"

    @staticmethod
    def no_priority(id_obstacle: int, direction_ego: OutgoingDirection, direction_other: OutgoingDirection) -> str:
        return f"No_{direction_ego}_{direction_other}_PriorityOver_V{id_obstacle}"

    @staticmethod
    def same_priority(id_obstacle: int, direction_ego: OutgoingDirection, direction_other: OutgoingDirection) -> str:
        return f"Same_{direction_ego}_{direction_other}_PriorityAs_V{id_obstacle}"

    # traffic lights-related
    @staticmethod
    def has_relevant_traffic_light() -> str:
        return "HasRelevantTrafficLight"

    @staticmethod
    def at_green_arrow() -> str:
        return "AtGreenArrow"

    @staticmethod
    def at_red_left_traffic_light(id_obstacle: int = None) -> str:
        suffix = 'AtRedLeft'
        if not id_obstacle:
            return suffix

        else:
            return f"V{str(id_obstacle)}_{suffix}"

    @staticmethod
    def at_red_straight_traffic_light(id_obstacle: int = None) -> str:
        suffix = 'AtRedStraight'
        if not id_obstacle:
            return suffix

        else:
            return f"V{str(id_obstacle)}_{suffix}"

    @staticmethod
    def at_red_right_traffic_light(id_obstacle: int = None) -> str:
        suffix = 'AtRedRight'
        if not id_obstacle:
            return suffix

        else:
            return f"V{str(id_obstacle)}_{suffix}"

    @staticmethod
    def not_endanger_mtl(id_obstacle: int, horizon: int = 20):
        o = id_obstacle
        return f"(({Proposition.causes_braking_for(o)} | F [0, {horizon}]({Proposition.in_conflict_by(o)}) | " \
               f"O [0, {horizon}]({Proposition.in_conflict_by(o)})) -> !{Proposition.in_conflict_with(o)})"

    @staticmethod
    def not_endanger_ctl(id_obstacle: int):
        o = id_obstacle
        return f"({Proposition.causes_braking_for(o)} -> !{Proposition.in_conflict_with(o)})"
