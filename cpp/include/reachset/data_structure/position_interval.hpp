#pragma once

#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include "reachset/utility/shared_include.hpp"

namespace reach {
/// Class to represent position intervals in which a set of propositions hold.
class PositionInterval {
public:
    double p_min{};
    double p_max{};
    std::set<std::string> set_propositions{};

    PositionInterval() = default;

    PositionInterval(double const& p_min, double const& p_max, std::set<std::string> const& set_propositions);

    explicit PositionInterval(py::handle obj_position_interval_py);

    inline PositionInterval clone() const {
        return PositionInterval{p_min, p_max, set_propositions};
    }

    bool intersects(double const& p_min, double const& p_max) const;
};

using PositionIntervalPtr = std::shared_ptr<PositionInterval>;
}
