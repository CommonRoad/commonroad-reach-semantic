#include "reach_semantic/data_structure/vehicle.hpp"

using namespace semantic_reach;

Vehicle::Vehicle(const pybind11::handle &obj_vehicle_py) : obj_vehicle_py(obj_vehicle_py) {
    vehicle_id = obj_vehicle_py.attr("id_vehicle").cast<int>();

    length = obj_vehicle_py.attr("shape").attr("length").cast<double>();

    for (const auto &lanelet_id: obj_vehicle_py.attr("lane").attr("list_ids_lanelets")) {
        lane_lanelet_ids.insert(lanelet_id.cast<int>());
    }

    pybind11::dict dict_step_to_state_lon_ref = obj_vehicle_py.attr("dict_step_to_state_lon_ref");
    for (const auto &[step, state_lon_ref]: dict_step_to_state_lon_ref) {
        if (state_lon_ref.is_none()) {
            continue;
        }
        map_step_to_state_lon_ref_s[step.cast<int>()] = state_lon_ref.attr("s").cast<double>();
    }
}

std::set<int> Vehicle::lanelet_ids_at_step(int step) {
    auto lanelet_ids_py = obj_vehicle_py.attr("lanelet_ids_at_step")(step);
    std::set<int> lanelet_ids{};
    for (const auto &lanelet_id: lanelet_ids_py) {
        lanelet_ids.insert(lanelet_id.cast<int>());
    }
    return lanelet_ids;
}

bool Vehicle::braking_caused_by_node_at_step(int step, const reach::ReachNodePtr &reachable_set) {
    return obj_vehicle_py.attr("braking_caused_by_node_at_step")(step, reachable_set).cast<bool>();
}
