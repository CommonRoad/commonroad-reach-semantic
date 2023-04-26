#pragma once

#include "reachset/utility/shared_include.hpp"

namespace reach {
/// Proposition groups.
enum class PropositionGroup {
    POSITION,
    VELOCITY,
    ACCELERATION,
    VEHICLE,
    TRAFFIC_SIGN,
    TRAFFIC_LIGHT,
    INTERSECTION,
    PRIORITY,
    TRAFFIC_STATUS,
    TEMPORARY
};
}