#include <utility>

#include "reach_semantic/data_structure/reach/reachable_set_labeler.hpp"
#include "reach_semantic/utility/reach_operation.hpp"

using namespace semantic_reach;

ReachableSetLabeler::ReachableSetLabeler(SemanticModelPtr semantic_model, SemanticConfigurationPtr config)
        : semantic_model(std::move(semantic_model)),
          config(std::move(config)),
          reachable_set_to_propositions(),
          reachable_set_to_lanelet_ids() {}

void ReachableSetLabeler::label_initial_state(const std::vector<reach::ReachNodePtr> &reachable_sets, int step_start) {
    for (const auto &reachable_set: reachable_sets) {
        auto drivable_area = reachable_set->position_rectangle();
        auto [propositions, set_ids_lanelets] = _obtain_propositions_for_rectangle(drivable_area, step_start);
        reachable_set_to_propositions[reachable_set] = propositions;
        reachable_set_to_lanelet_ids[reachable_set] = set_ids_lanelets;
    }
    label_traffic_propositions(step_start, reachable_sets);
}

std::pair<PropositionHolder, std::set<int>>
ReachableSetLabeler::_obtain_propositions_for_rectangle(const reach::ReachPolygonPtr &rectangle, int step) {
    auto proposition_holder = PropositionHolder();
    std::set<int> set_ids_lanelets{};
    // retrieve propositions from the intersecting lanelet region
    for (const auto &region: semantic_model->vec_regions) {
        if (region->polygon_cvln->intersects(rectangle)) {
            for (const auto &[group, set_propositions]: region->map_group_to_propositions_at_step(step)) {
                proposition_holder.add_propositions(set_propositions, group);
            }
            set_ids_lanelets.insert(region->set_ids_lanelets.begin(), region->set_ids_lanelets.end());
            break;
        }
    }

    // retrieve vehicle-related propositions from position intervals
    auto list_intervals_lon = semantic_model->map_step_to_position_intervals[step]["lon"];
    auto list_intervals_lat = semantic_model->map_step_to_position_intervals[step]["lat"];

    for (const auto &interval_lon: list_intervals_lon) {
        if (interval_lon->intersects(rectangle->p_lon_min(), rectangle->p_lon_max())) {
            proposition_holder.add_propositions(interval_lon->set_propositions, PropositionGroup::POSITION);
            break;
        }
    }

    for (const auto &interval_lat: list_intervals_lat) {
        if (interval_lat->intersects(rectangle->p_lat_min(), rectangle->p_lat_max())) {
            proposition_holder.add_propositions(interval_lat->set_propositions, PropositionGroup::POSITION);
            break;
        }
    }

    return {proposition_holder, set_ids_lanelets};
}

std::vector<reach::ReachNodePtr>
ReachableSetLabeler::label_traffic_propositions(int step, std::vector<reach::ReachNodePtr> reachable_sets) {
    if (reachable_sets.empty()) {
        return {};
    }

    reachable_sets = _label_traffic_status_propositions(step, reachable_sets);
    reachable_sets = _label_in_conflict_area_propositions(step, reachable_sets);
    reachable_sets = _label_causes_braking_propositions(step, reachable_sets);

    return reachable_sets;
}

std::vector<reach::ReachNodePtr>
ReachableSetLabeler::_label_traffic_status_propositions(int step, std::vector<reach::ReachNodePtr> reachable_sets) {
    auto set_propositions = semantic_model->map_step_to_traffic_status_propositions[step];
    for (const auto &reachable_set: reachable_sets) {
        reachable_set_to_propositions[reachable_set].add_propositions(set_propositions,
                                                                      PropositionGroup::TRAFFIC_STATUS);
    }
    return reachable_sets;
}

std::vector<reach::ReachNodePtr>
ReachableSetLabeler::_label_in_conflict_area_propositions(int step, std::vector<reach::ReachNodePtr> reachable_sets) {
    // examine if the propagated set is conflicting with the vehicles
    for (const auto &reachable_set: reachable_sets) {
        for (const auto &vehicle: semantic_model->vec_vehicles) {
            for (const auto &id_lanelet_propagated_set: reachable_set_to_lanelet_ids[reachable_set]) {
                auto intersecting_lanelets = semantic_model->map_id_lanelet_to_set_ids_lanelets_intersecting[id_lanelet_propagated_set];
                for (const auto &id_lanelet_lane_vehicle: vehicle->lane_lanelet_ids) {
                    if (intersecting_lanelets.find(id_lanelet_lane_vehicle) != intersecting_lanelets.end()) {
                        reachable_set_to_propositions[reachable_set].add_proposition(
                                Proposition::in_conflict_with(vehicle->vehicle_id),
                                PropositionGroup::TRAFFIC_STATUS);
                    }
                }
            }
        }
    }

    // examine if the vehicles are in conflict with the propagated set
    for (const auto &reachable_set: reachable_sets) {
        for (const auto &vehicle: semantic_model->vec_vehicles) {
            double p_lon_min_reachable_set = reachable_set->p_lon_min() - config->semantic_model().ego_radius_inflation;
            // if step is not in map continue
            auto it = vehicle->map_step_to_state_lon_ref_s.find(step);
            if (it == vehicle->map_step_to_state_lon_ref_s.end()) {
                continue;
            }
            double p_lon_ref_max_vehicle = it->second + vehicle->length / 2;

            // propagated set is in front of the vehicle along the reference path
            if (p_lon_min_reachable_set > p_lon_ref_max_vehicle) {
                continue;
            }

            // iterate through lanelet ids of the route and lanelet ids of the vehicle
            for (const auto &id_lanelet_route: config->semantic_model().vec_route_lanelet_ids) {
                auto intersecting_lanelets = semantic_model->map_id_lanelet_to_set_ids_lanelets_intersecting[id_lanelet_route];
                for (const auto &id_lanelet_vehicle: vehicle->lanelet_ids_at_step(step)) {
                    if (intersecting_lanelets.find(id_lanelet_vehicle) != intersecting_lanelets.end()) {
                        reachable_set_to_propositions[reachable_set].add_proposition(
                                Proposition::in_conflict_by(vehicle->vehicle_id),
                                PropositionGroup::TRAFFIC_STATUS);
                    }
                }
            }
        }
    }

    return reachable_sets;
}

std::vector<reach::ReachNodePtr>
ReachableSetLabeler::_label_causes_braking_propositions(int step, std::vector<reach::ReachNodePtr> reachable_sets) {
    if (!config->semantic_model().is_intersection) {
        return reachable_sets;
    }

    for (const auto &reachable_set: reachable_sets) {
        for (const auto &vehicle: semantic_model->vec_vehicles) {
            if (vehicle->braking_caused_by_node_at_step(step, reachable_set)) {
                reachable_set_to_propositions[reachable_set].add_proposition(
                        Proposition::causes_braking_for(vehicle->vehicle_id),
                        PropositionGroup::TRAFFIC_STATUS);
            }
        }
    }
    return reachable_sets;
}

std::vector<reach::ReachNodePtr>
ReachableSetLabeler::split_wrt_regions(int step, const std::vector<reach::ReachNodePtr> &reachable_sets) {
    vector<reach::ReachNodePtr> vec_nodes_split = {};
    // iterate through region and examine propagated sets that are intersecting with the region
    for (const auto &reachable_set: reachable_sets) {
        for (auto const &region: semantic_model->vec_regions) {

            auto rectangle = reachable_set->position_rectangle();
            // there is no possibility of intersection
            if (!region->intersects(rectangle, "CVLN")) {
                continue;
            }

            // there is a possibility of intersection
            auto polygon_intersected = region->polygon_cvln->clone();
            // compute intersection with the position rectangle
            // TODO: Find out, why there was a try-catch here
            polygon_intersected->intersect_halfspace(1, 0, rectangle->p_lon_max());
            polygon_intersected->intersect_halfspace(-1, 0, -rectangle->p_lon_min());
            polygon_intersected->intersect_halfspace(0, 1, rectangle->p_lat_max());
            polygon_intersected->intersect_halfspace(0, -1, -rectangle->p_lat_min());

            if (polygon_intersected->empty()) {
                continue;
            }

            // over-approximate by restoring to axis-aligned rectangles
            auto [p_lon_min, p_lat_min, p_lon_max, p_lat_max] = polygon_intersected->bounding_box();

            // TODO: Find out, why there was a try-catch here
            // clone the propagated set and split in the position domain, update the propositions
            auto node_new = reachable_set->clone();
            reachable_set_to_propositions[node_new] = reachable_set_to_propositions[reachable_set].clone();
            node_new->intersect_in_position_domain(p_lon_min, p_lat_min, p_lon_max, p_lat_max);
            vec_nodes_split.emplace_back(_update_propositions_with_region(node_new, region, step));
        }
    }

    return vec_nodes_split;
}

reach::ReachNodePtr
ReachableSetLabeler::_update_propositions_with_region(reach::ReachNodePtr propagated_set,
                                                      const semantic_reach::RegionPtr &region, int step) {
    auto relevant_props = region->map_group_to_propositions_at_step(step);
    for (const auto &[group, set_propositions]: relevant_props) {
        reachable_set_to_propositions[propagated_set].add_propositions(set_propositions, group);
    }

    // add lanelet ids of the region to propagated set
    reachable_set_to_lanelet_ids[propagated_set].insert(region->set_ids_lanelets.begin(),
                                                        region->set_ids_lanelets.end());

    // add lanelet transition as temporary propositions
    auto set_propositions = _obtain_lanelet_transition_propositions(propagated_set);
    reachable_set_to_propositions[propagated_set].add_propositions(set_propositions, PropositionGroup::TEMPORARY);

    return propagated_set;
}

std::vector<reach::ReachNodePtr>
ReachableSetLabeler::split_wrt_position_intervals(int step, const std::vector<reach::ReachNodePtr> &reachable_sets) {
    auto vec_intervals_lon = semantic_model->map_step_to_position_intervals[step]["lon"];
    auto vec_intervals_lat = semantic_model->map_step_to_position_intervals[step]["lat"];

    vector<reach::ReachNodePtr> vec_nodes_split = {};

    for (const auto &reachable_set: reachable_sets) {
        // longitudinal direction
        auto vec_nodes_split_lon = _split_reachable_set_wrt_intervals(reachable_set, vec_intervals_lon,
                                                                      reachable_set->p_lon_min(),
                                                                      reachable_set->p_lon_max(), "lon");

        // lateral direction
        for (auto const &node: vec_nodes_split_lon) {
            auto vec_nodes_split_lat = _split_reachable_set_wrt_intervals(node, vec_intervals_lat, node->p_lat_min(),
                                                                          node->p_lat_max(), "lat");
            vec_nodes_split.insert(vec_nodes_split.end(), vec_nodes_split_lat.begin(), vec_nodes_split_lat.end());
        }
    }

    return vec_nodes_split;
}

std::vector<reach::ReachNodePtr>
ReachableSetLabeler::_split_reachable_set_wrt_intervals(const reach::ReachNodePtr &reachable_set,
                                                        const std::vector<PositionIntervalPtr> &intervals,
                                                        double reach_min,
                                                        double reach_max, const std::string &direction) {
    vector<reach::ReachNodePtr> vec_nodes_split = {};
    for (auto const &interval: intervals) {
        if (interval->intersects(reach_min, reach_max)) {
            auto node_split = semantic_reach::split_reach_node_wrt_interval(reachable_set, interval, direction);
            if (node_split) {
                copy_labels(reachable_set, {node_split});
                reachable_set_to_propositions[node_split].add_propositions(interval->set_propositions,
                                                                           PropositionGroup::POSITION);
                vec_nodes_split.emplace_back(node_split);
            }
        } else if (interval->p_min > reach_max) {
            // early termination, since the rest of intervals will definitely not intersect with the reachable set
            break;
        }
    }
    return vec_nodes_split;
}

std::set<std::string>
ReachableSetLabeler::_obtain_lanelet_transition_propositions(
        const reach::ReachNodePtr &propagated_set) {
    auto source_node = propagated_set->vec_nodes_source[0];

    auto set_propositions_position_source = reachable_set_to_propositions[source_node].propositions_in_group(
            PropositionGroup::POSITION);
    auto source_lanelets = std::set<int>{};
    for (const auto &proposition: set_propositions_position_source) {
        if (proposition.find(Proposition::in_lanelet()) != std::string::npos) {
            int lanelet_id = std::stoi(proposition.substr(proposition.find('_') + 1));
            source_lanelets.insert(lanelet_id);
        }
    }

    // generate lanelet transition propositions
    auto set_propositions = std::set<std::string>{};
    for (const auto &source_lanelet_id: source_lanelets) {
        for (const auto &base_set_lanelet_id: reachable_set_to_lanelet_ids[source_node]) {
            if (source_lanelet_id != base_set_lanelet_id) {
                set_propositions.insert(Proposition::lanelet_transition(source_lanelet_id, base_set_lanelet_id));
            }
        }
    }

    return set_propositions;
}

std::vector<reach::ReachNodePtr>
ReachableSetLabeler::discard_colliding_nodes(const std::vector<reach::ReachNodePtr> &reachable_sets) {
    std::vector<reach::ReachNodePtr> vec_nodes_keep{};

    for (const auto &reachable_set: reachable_sets) {
        bool colliding = false;
        auto set_propositions = reachable_set_to_propositions[reachable_set].set_propositions;
        for (const auto &proposition: set_propositions) {
            if (proposition.find(Proposition::aligned_with()) != std::string::npos) {
                int vehicle_id = std::stoi(proposition.substr(proposition.find('_') + 2));
                std::string x = Proposition::beside(vehicle_id);
                if (set_propositions.find(x) != set_propositions.end()) {
                    colliding = true;
                    break;
                }
            }
        }
        if (!colliding) {
            vec_nodes_keep.emplace_back(reachable_set);
        }
    }

    return vec_nodes_keep;
}

void ReachableSetLabeler::copy_labels(const reach::ReachNodePtr &source_reachable_set,
                                      const std::vector<reach::ReachNodePtr> &reachable_sets) {
    for (const auto &reachable_set: reachable_sets) {
        reachable_set_to_propositions[reachable_set] = reachable_set_to_propositions[source_reachable_set].clone();
        reachable_set_to_lanelet_ids[reachable_set] = reachable_set_to_lanelet_ids[source_reachable_set];
    }
}
