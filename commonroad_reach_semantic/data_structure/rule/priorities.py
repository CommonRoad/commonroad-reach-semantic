from typing import Dict

import numpy as np
from commonroad.scenario.traffic_sign import TrafficSignIDZamunda

from commonroad_reach_semantic.data_structure.config.outgoing_direction import OutgoingDirection

# TODO: Handle index somewhere else so that the inner dict is a Dict[OutgoingDirection, int]
dict_traffic_sign_to_priorities: Dict[TrafficSignIDZamunda, Dict[str, float]] = {
    TrafficSignIDZamunda.ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_RIGHT_YIELD: {
        OutgoingDirection.LEFT: 5,
        OutgoingDirection.STRAIGHT: 4,
        OutgoingDirection.RIGHT: 4,
        "index": 1,
    },
    TrafficSignIDZamunda.ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD: {
        OutgoingDirection.LEFT: 5,
        OutgoingDirection.STRAIGHT: 4,
        OutgoingDirection.RIGHT: -np.inf,
        "index": 2,
    },
    TrafficSignIDZamunda.ADDITION_LEFT_TURNING_PRIORITY_WITH_RIGHT_YIELD: {
        OutgoingDirection.LEFT: 5,
        OutgoingDirection.STRAIGHT: -np.inf,
        OutgoingDirection.RIGHT: 4,
        "index": 3,
    },
    TrafficSignIDZamunda.ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_LEFT_YIELD: {
        OutgoingDirection.LEFT: 4,
        OutgoingDirection.STRAIGHT: 4,
        OutgoingDirection.RIGHT: 5,
        "index": 4,
    },
    TrafficSignIDZamunda.ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD: {
        OutgoingDirection.LEFT: -np.inf,
        OutgoingDirection.STRAIGHT: 4,
        OutgoingDirection.RIGHT: 5,
        "index": 5,
    },
    TrafficSignIDZamunda.ADDITION_RIGHT_TURNING_PRIORITY_WITH_LEFT_YIELD: {
        OutgoingDirection.LEFT: 4,
        OutgoingDirection.STRAIGHT: -np.inf,
        OutgoingDirection.RIGHT: 5,
        "index": 6,
    },
    TrafficSignIDZamunda.ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_RIGHT_YIELD: {
        OutgoingDirection.LEFT: 2,
        OutgoingDirection.STRAIGHT: 2,
        OutgoingDirection.RIGHT: 2,
        "index": 7,
    },
    TrafficSignIDZamunda.ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD: {
        OutgoingDirection.LEFT: 2,
        OutgoingDirection.STRAIGHT: 2,
        OutgoingDirection.RIGHT: -np.inf,
        "index": 8,
    },
    TrafficSignIDZamunda.ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_LEFT_YIELD: {
        OutgoingDirection.LEFT: 2,
        OutgoingDirection.STRAIGHT: 2,
        OutgoingDirection.RIGHT: 2,
        "index": 9,
    },
    TrafficSignIDZamunda.ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD: {
        OutgoingDirection.LEFT: -np.inf,
        OutgoingDirection.STRAIGHT: 2,
        OutgoingDirection.RIGHT: 2,
        "index": 10,
    },
    TrafficSignIDZamunda.PRIORITY: {
        OutgoingDirection.LEFT: 4,
        OutgoingDirection.STRAIGHT: 5,
        OutgoingDirection.RIGHT: 4,
        "index": 11,
    },
    TrafficSignIDZamunda.RIGHT_OF_WAY: {
        OutgoingDirection.LEFT: 4,
        OutgoingDirection.STRAIGHT: 5,
        OutgoingDirection.RIGHT: 4,
        "index": 12,
    },
    TrafficSignIDZamunda.YIELD: {
        OutgoingDirection.LEFT: 2,
        OutgoingDirection.STRAIGHT: 2,
        OutgoingDirection.RIGHT: 2,
        "index": 13,
    },
    TrafficSignIDZamunda.STOP: {
        OutgoingDirection.LEFT: 1,
        OutgoingDirection.STRAIGHT: 1,
        OutgoingDirection.RIGHT: 1,
        "index": 14,
    },
    TrafficSignIDZamunda.WARNING_RIGHT_BEFORE_LEFT: {
        OutgoingDirection.LEFT: 3,
        OutgoingDirection.STRAIGHT: 3,
        OutgoingDirection.RIGHT: 3,
        "index": 15,
    },
    TrafficSignIDZamunda.GREEN_ARROW: {
        OutgoingDirection.LEFT: -np.inf,
        OutgoingDirection.STRAIGHT: -np.inf,
        OutgoingDirection.RIGHT: 0,
        "index": 16,
    },
}
