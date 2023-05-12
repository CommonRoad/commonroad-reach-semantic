#include "reach_semantic/data_structure/semantic_model.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace semantic_reach;

SemanticModel::SemanticModel(py::handle const &obj_semantic_model_py) {
    this->obj_semantic_model_py = obj_semantic_model_py;

    // regions
    for (auto const &obj_region_py: obj_semantic_model_py.attr("region_model").attr("list_regions")) {
        vec_regions.emplace_back(make_shared<Region>(obj_region_py));
    }
    // position intervals
    auto dict_step_to_position_intervals =
            obj_semantic_model_py.attr("vehicle_model").attr("dict_step_to_position_intervals");
    for (auto const &step: dict_step_to_position_intervals) {
        for (auto const &direction: dict_step_to_position_intervals[step]) {
            for (auto const &position_interval_py: dict_step_to_position_intervals[step][direction]) {
                map_step_to_position_intervals[step.cast<int>()][direction.cast<string>()].emplace_back(
                        make_shared<PositionInterval>(position_interval_py));
            }
        }
    }
    // traffic status propositions
    py::dict dict_step_to_traffic_status_propositions =
            obj_semantic_model_py.attr("traffic_status_model").attr("dict_step_to_traffic_status_propositions");
    for (auto const &[step, propositions]: dict_step_to_traffic_status_propositions) {
        map_step_to_traffic_status_propositions[step.cast<int>()] = dict_step_to_traffic_status_propositions[step].cast<set<string>>();
    }
}

std::set<int>
SemanticModel::get_braking_vehicle_ids(int step, const semantic_reach::SemanticReachNodePtr &reachable_set) {
    std::set<int> braking_vehicle_ids{};
    for (auto const &vehicle: obj_semantic_model_py.attr("vehicle_model").attr("list_vehicles")) {
        if (vehicle.attr("braking_caused_by_node_at_step")(step, reachable_set).cast<bool>()) {
            braking_vehicle_ids.insert(vehicle.attr("id_vehicle").cast<int>());
        }
    }
    return braking_vehicle_ids;
}