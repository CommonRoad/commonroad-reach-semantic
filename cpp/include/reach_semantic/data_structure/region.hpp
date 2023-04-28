#pragma once

#include "reach_semantic/utility/shared_include.hpp"
#include "reachset/data_structure/reach/reach_polygon.hpp"
#include "reach_semantic/data_structure/proposition_holder.hpp"

namespace semantic_reach {
/// Class to represent a lanelet region in the scenario.
class Region {
public:
    int step_end{};
    std::set<int> set_ids_lanelets{};
    reach::ReachPolygonPtr polygon_cart;
    reach::ReachPolygonPtr polygon_cvln;
    MultiStepPropositionHolderPtr proposition_holder;

    Region() = default;

    explicit Region(py::handle const& obj_region_py);

    /// Returns true if the input box intersects with the bounding box.
    bool intersects(reach::ReachPolygonPtr const& coordinates_box, std::string const& coordinate_system = "CVLN") const;

    inline auto propositions_at_step(int const& step){
        return proposition_holder->propositions_at_step(step);
    }

    inline auto map_group_to_propositions_at_step(int const& step) {
        return proposition_holder->map_group_to_propositions_at_step(step);
    }
};

using RegionPtr = std::shared_ptr<Region>;
}