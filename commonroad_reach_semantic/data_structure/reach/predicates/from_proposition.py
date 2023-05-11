import re

from commonroad_reach_semantic.data_structure.reach.predicates.behind_obstacle_predicate import BehindObstaclePredicate
from commonroad_reach_semantic.data_structure.reach.predicates.beside_obstacle_predicate import BesideObstaclePredicate
from commonroad_reach_semantic.data_structure.reach.predicates.causes_braking_predicate import CausesBrakingPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.in_conflict_area_of_vehicle_predicate import \
    InConflictAreaOfVehiclePredicate
from commonroad_reach_semantic.data_structure.reach.predicates.in_front_of_obstacle_predicate import \
    InFrontOfObstaclePredicate
from commonroad_reach_semantic.data_structure.reach.predicates.in_intersection_predicate import InIntersectionPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.in_lanelet_predicate import InLaneletPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.in_straight_successor_predicate import \
    InStraightSuccessorPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.vehicle_in_conflict_area_predicate import \
    VehicleInConflictAreaPredicate


def from_proposition(proposition: str):
    if matched := re.fullmatch(r"InLanelet_(\d+)", proposition):
        return InLaneletPredicate(int(matched.group(1)))
    elif matched := re.fullmatch(r"Behind_V(\d+)", proposition):
        return BehindObstaclePredicate(int(matched.group(1)))
    elif matched := re.fullmatch(r"Beside_V(\d+)", proposition):
        return BesideObstaclePredicate(int(matched.group(1)))
    elif matched := re.fullmatch(r"InFrontOf_V(\d+)", proposition):
        return InFrontOfObstaclePredicate(int(matched.group(1)))
    elif re.fullmatch(r"InStraightSuc", proposition):
        return InStraightSuccessorPredicate()
    elif re.fullmatch(r"InIntersection", proposition):
        return InIntersectionPredicate()
    elif matched := re.fullmatch(r"CausesBrakingFor_V(\d+)", proposition):
        return CausesBrakingPredicate(int(matched.group(1)))
    elif matched := re.fullmatch(r"InConflictWith_V(\d+)", proposition):
        return InConflictAreaOfVehiclePredicate(int(matched.group(1)))
    elif matched := re.fullmatch(r"InConflictBy_V(\d+)", proposition):
        return VehicleInConflictAreaPredicate(int(matched.group(1)))
    else:
        raise ValueError(f"Unknown proposition: {proposition}")