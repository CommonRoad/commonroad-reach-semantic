#pragma once

#include "reach_semantic/data_structure/environment_model/semantic_model.hpp"
#include "reach_semantic/data_structure/region.hpp"

#include "reachset/data_structure/reach/reach_node.hpp"

namespace semantic_reach {
/**
 * Splits reach nodes according to regions in the semantic model.
 */
class RegionReachNodeSplitter {
  private:
    SemanticModelPtr semantic_model;

  public:
    /**
     * Create a new splitter for the given semantic model.
     *
     * @param semantic_model The semantic model.
     */
    explicit RegionReachNodeSplitter(SemanticModelPtr semantic_model);

    /**
     * Splits a reachable set with respect to lanelet regions.
     *
     * Steps:
     *     1. Intersect reachable set in the position domain with lanelet regions
     *     2. Over-approximate and restore to axis-aligned rectangles
     *
     * @param reachable_set The reachable set to split.
     * @returns A list of pairs of regions and the corresponding reachable set.
     */
    std::vector<std::pair<RegionPtr, reach::ReachNodePtr>> split_wrt_regions(const reach::ReachNodePtr &reachable_set);
};
} // namespace semantic_reach
