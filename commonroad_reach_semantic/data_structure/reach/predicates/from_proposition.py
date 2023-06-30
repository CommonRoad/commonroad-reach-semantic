import re

from commonroad_reach_semantic.data_structure.config.outgoing_direction import OutgoingDirection
from commonroad_reach_semantic.data_structure.reach.predicates.aligned_with_obstacle_predicate import \
    AlignedWithObstaclePredicate
from commonroad_reach_semantic.data_structure.reach.predicates.behind_obstacle_predicate import BehindObstaclePredicate
from commonroad_reach_semantic.data_structure.reach.predicates.beside_obstacle_predicate import BesideObstaclePredicate
from commonroad_reach_semantic.data_structure.reach.predicates.causes_braking_predicate import CausesBrakingPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.drives_backward_predicate import DrivesBackwardPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.has_priority_predicate import HasPriorityPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.in_conflict_area_of_vehicle_predicate import \
    InConflictAreaOfVehiclePredicate
from commonroad_reach_semantic.data_structure.reach.predicates.in_front_of_obstacle_predicate import \
    InFrontOfObstaclePredicate
from commonroad_reach_semantic.data_structure.reach.predicates.in_intersection_predicate import InIntersectionPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.in_lanelet_predicate import InLaneletPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.in_straight_successor_predicate import \
    InStraightSuccessorPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.left_of_obstacle_predicate import LeftOfObstaclePredicate
from commonroad_reach_semantic.data_structure.reach.predicates.no_priority_predicate import NoPriorityPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.right_of_obstacle_predicate import \
    RightOfObstaclePredicate
from commonroad_reach_semantic.data_structure.reach.predicates.same_priority_predicate import SamePriorityPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.vehicle_in_conflict_area_predicate import \
    VehicleInConflictAreaPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.vehicle_in_lanelet_predicate import \
    VehicleInLaneletPredicate
from commonroad_reach_semantic.data_structure.reach.predicates.vehicle_in_successor_predicate import \
    VehicleInSuccessorPredicate


def from_proposition(proposition: str, negated: bool):
    if matched := re.fullmatch(r"InLanelet_(\d+)", proposition):
        return InLaneletPredicate(int(matched.group(1)), negated)
    elif matched := re.fullmatch(r"InLanelet_(\d+)_V(\d+)", proposition):
        return VehicleInLaneletPredicate(int(matched.group(2)), int(matched.group(1)), negated)
    elif matched := re.fullmatch(r"Behind_V(\d+)", proposition):
        return BehindObstaclePredicate(int(matched.group(1)), negated)
    elif matched := re.fullmatch(r"Beside_V(\d+)", proposition):
        return BesideObstaclePredicate(int(matched.group(1)), negated)
    elif matched := re.fullmatch(r"InFrontOf_V(\d+)", proposition):
        return InFrontOfObstaclePredicate(int(matched.group(1)), negated)
    elif matched := re.fullmatch(r"RightOf_V(\d+)", proposition):
        return RightOfObstaclePredicate(int(matched.group(1)), negated)
    elif matched := re.fullmatch(r"AlignedWith_V(\d+)", proposition):
        return AlignedWithObstaclePredicate(int(matched.group(1)), negated)
    elif matched := re.fullmatch(r"LeftOf_V(\d+)", proposition):
        return LeftOfObstaclePredicate(int(matched.group(1)), negated)
    elif re.fullmatch(r"InStraightSuc", proposition):
        return InStraightSuccessorPredicate(negated)
    elif matched := re.fullmatch(r"V(\d+)_In(Left|Straight|Right)Suc", proposition):
        return VehicleInSuccessorPredicate(int(matched.group(1)), OutgoingDirection[matched.group(2).upper()], negated)
    elif re.fullmatch(r"InIntersection", proposition):
        return InIntersectionPredicate(negated)
    elif re.fullmatch(r"DrivesBackward", proposition):
        return DrivesBackwardPredicate(negated)
    elif matched := re.fullmatch(r"CausesBrakingFor_V(\d+)", proposition):
        return CausesBrakingPredicate(int(matched.group(1)), negated)
    elif matched := re.fullmatch(r"InConflictWith_V(\d+)", proposition):
        return InConflictAreaOfVehiclePredicate(int(matched.group(1)), negated)
    elif matched := re.fullmatch(r"InConflictBy_V(\d+)", proposition):
        return VehicleInConflictAreaPredicate(int(matched.group(1)), negated)
    elif matched := re.fullmatch(r"Has_(left|straight|right)_(left|straight|right)_PriorityOver_V(\d+)", proposition):
        return HasPriorityPredicate(int(matched.group(3)), OutgoingDirection[matched.group(1).upper()],
                                    OutgoingDirection[matched.group(2).upper()], negated)
    elif matched := re.fullmatch(r"No_(left|straight|right)_(left|straight|right)_PriorityOver_V(\d+)", proposition):
        return NoPriorityPredicate(int(matched.group(3)), OutgoingDirection[matched.group(1).upper()],
                                   OutgoingDirection[matched.group(2).upper()], negated)
    elif matched := re.fullmatch(r"Same_(left|straight|right)_(left|straight|right)_PriorityAs_V(\d+)", proposition):
        return SamePriorityPredicate(int(matched.group(3)), OutgoingDirection[matched.group(1).upper()],
                                     OutgoingDirection[matched.group(2).upper()], negated)
    else:
        raise ValueError(f"Unknown proposition: {proposition}")
