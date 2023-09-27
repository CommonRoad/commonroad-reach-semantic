#include "reach_semantic/data_structure/reach/splitters/region_reach_node_splitter.hpp"

using namespace semantic_reach;

RegionReachNodeSplitter::RegionReachNodeSplitter(semantic_reach::SemanticModelPtr semantic_model)
    : semantic_model(std::move(semantic_model)) {}

std::vector<std::pair<RegionPtr, reach::ReachNodePtr>>
RegionReachNodeSplitter::split_wrt_regions(const reach::ReachNodePtr &reachable_set) {
    std::vector<std::pair<RegionPtr, reach::ReachNodePtr>> vec_nodes_split{};
    // iterate through region and examine propagated sets that are intersecting with the region
    for (auto const &region : semantic_model->vec_regions) {
        // first compute intersection with bounding box
        // --> exact intersection is more expensive, so we only want to compute it if necessary
        // there is no possibility of intersection
        if (!region->bounding_box_intersects(reachable_set, "CVLN")) {
            continue;
        }

        // there is a possibility of intersection
        auto polygon_intersected = region->polygon_cvln->clone();
        // compute intersection with the position rectangle
        polygon_intersected->intersect_halfspace(1, 0, reachable_set->p_lon_max());
        polygon_intersected->intersect_halfspace(-1, 0, -reachable_set->p_lon_min());
        polygon_intersected->intersect_halfspace(0, 1, reachable_set->p_lat_max());
        polygon_intersected->intersect_halfspace(0, -1, -reachable_set->p_lat_min());

        if (polygon_intersected->empty()) {
            continue;
        }

        auto restricted_reachable_set = reachable_set->clone();
        // intersect the reachable set with the polygon
        // as we can only represent axis-aligned rectangles, we have to overapproximate by using the polygon bounds
        auto [p_lon_min, p_lat_min, p_lon_max, p_lat_max] = polygon_intersected->bounding_box();
        restricted_reachable_set->intersect_in_position_domain(p_lon_min, p_lat_min, p_lon_max, p_lat_max);
        vec_nodes_split.emplace_back(region, restricted_reachable_set);
    }

    return vec_nodes_split;
}
