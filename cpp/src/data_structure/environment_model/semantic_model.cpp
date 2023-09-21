#include "reach_semantic/data_structure/environment_model/semantic_model.hpp"
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
    // intersecting lanelets
    py::dict dict_id_lanelet_to_set_ids_lanelets_intersecting =
            obj_semantic_model_py.attr("lanelet_model").attr("dict_id_lanelet_to_set_ids_lanelets_intersecting");
    for (auto const &[id_lanelet, set_ids_lanelets_intersecting]: dict_id_lanelet_to_set_ids_lanelets_intersecting) {
        map_id_lanelet_to_set_ids_lanelets_intersecting[id_lanelet.cast<int>()] = dict_id_lanelet_to_set_ids_lanelets_intersecting[id_lanelet].cast<set<int>>();
    }
    // vehicles
    for (auto const &obj_vehicle_py: obj_semantic_model_py.attr("vehicle_model").attr("list_vehicles")) {
        vec_vehicles.emplace_back(make_shared<Vehicle>(obj_vehicle_py));
    }
}
