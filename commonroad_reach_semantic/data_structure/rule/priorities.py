from typing import Dict

import numpy as np
from commonroad.scenario.traffic_sign import TrafficSignIDZamunda

dict_traffic_sign_to_priorities: Dict[TrafficSignIDZamunda, Dict[str, float]] = {
    TrafficSignIDZamunda.ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_RIGHT_YIELD: {"left": 5, "straight": 4,
                                                                                    "right": 4, "index": 1},
    TrafficSignIDZamunda.ADDITION_LEFT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD: {"left": 5, "straight": 4,
                                                                              "right": -np.inf, "index": 2},
    TrafficSignIDZamunda.ADDITION_LEFT_TURNING_PRIORITY_WITH_RIGHT_YIELD: {"left": 5, "straight": -np.inf,
                                                                           "right": 4, "index": 3},
    TrafficSignIDZamunda.ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_LEFT_YIELD: {"left": 4, "straight": 4,
                                                                                    "right": 5, "index": 4},
    TrafficSignIDZamunda.ADDITION_RIGHT_TURNING_PRIORITY_WITH_OPPOSITE_YIELD: {"left": -np.inf, "straight": 4,
                                                                               "right": 5, "index": 5},
    TrafficSignIDZamunda.ADDITION_RIGHT_TURNING_PRIORITY_WITH_LEFT_YIELD: {"left": 4, "straight": -np.inf,
                                                                           "right": 5, "index": 6},
    TrafficSignIDZamunda.ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_RIGHT_YIELD: {"left": 2, "straight": 2,
                                                                                    "right": 2, "index": 7},
    TrafficSignIDZamunda.ADDITION_LEFT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD: {"left": 2, "straight": 2,
                                                                              "right": -np.inf, "index": 8},
    TrafficSignIDZamunda.ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_LEFT_YIELD: {"left": 2, "straight": 2,
                                                                                    "right": 2, "index": 9},
    TrafficSignIDZamunda.ADDITION_RIGHT_TRAFFIC_PRIORITY_WITH_STRAIGHT_YIELD: {"left": -np.inf, "straight": 2,
                                                                               "right": 2, "index": 10},
    TrafficSignIDZamunda.PRIORITY: {"left": 4, "straight": 5, "right": 4, "index": 11},
    TrafficSignIDZamunda.RIGHT_OF_WAY: {"left": 4, "straight": 5, "right": 4, "index": 12},
    TrafficSignIDZamunda.YIELD: {"left": 2, "straight": 2, "right": 2, "index": 13},
    TrafficSignIDZamunda.STOP: {"left": 1, "straight": 1, "right": 1, "index": 14},
    TrafficSignIDZamunda.WARNING_RIGHT_BEFORE_LEFT: {"left": 3, "straight": 3, "right": 3, "index": 15},
    TrafficSignIDZamunda.GREEN_ARROW: {"left": -np.inf, "straight": -np.inf, "right": 0, "index": 16},
}
