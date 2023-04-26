#pragma once

#include "reachset/utility/shared_include.hpp"
#include "reachset/data_structure/reach/reach_polygon_boost.hpp"
#include "reachset/data_structure/proposition_holder.hpp"

namespace reach {
/// Class to represent a lanelet region in the scenario.
class Region {
public:
    int step_end{};
    std::set<int> set_ids_lanelets{};
    ReachPolygonPtr polygon_cart;
    ReachPolygonPtr polygon_cvln;
    MultiStepPropositionHolderPtr proposition_holder;

    Region() = default;

    explicit Region(py::handle const& obj_region_py);

    /// Returns true if the input box intersects with the bounding box.
    bool intersects(ReachPolygonPtr const& coordinates_box, std::string const& coordinate_system = "CVLN") const;

    inline auto propositions_at_step(int const& step){
        return proposition_holder->propositions_at_step(step);
    }

    inline auto map_group_to_propositions_at_step(int const& step) {
        return proposition_holder->map_group_to_propositions_at_step(step);
    }
};

using RegionPtr = std::shared_ptr<Region>;
}