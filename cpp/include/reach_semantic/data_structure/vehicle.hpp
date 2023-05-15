#pragma once

#include <pybind11/embed.h>

#include "reachset/data_structure/reach/reach_node.hpp"

namespace semantic_reach {
    /// Class to represent a vehicle in a CommonRoad scenario.
    class Vehicle {
    private:
        pybind11::handle obj_vehicle_py;

    public:
        int vehicle_id;

        double length;

        std::set<int> lane_lanelet_ids;

        std::map<int, double> map_step_to_state_lon_ref_s;

        explicit Vehicle(const pybind11::handle &obj_vehicle_py);

        std::set<int> lanelet_ids_at_step(int step);

        bool braking_caused_by_node_at_step(int step, const reach::ReachNodePtr &reachable_set);
    };

    using VehiclePtr = std::shared_ptr<Vehicle>;
}
