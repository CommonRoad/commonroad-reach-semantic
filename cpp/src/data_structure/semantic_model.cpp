#include "reachset/data_structure/semantic_model.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace reach;

SemanticModel::SemanticModel(py::handle const& obj_semantic_model_py) {
    this->obj_semantic_model_py = obj_semantic_model_py;

    // regions
    for (auto const& obj_region_py: obj_semantic_model_py.attr("list_regions")) {
        vec_regions.emplace_back(make_shared<Region>(obj_region_py));
    }
    // position intervals
    auto dict_step_to_position_intervals =
            obj_semantic_model_py.attr("dict_step_to_position_intervals");
    for (auto const& step: dict_step_to_position_intervals) {
        for (auto const& direction: dict_step_to_position_intervals[step]) {
            for (auto const& position_interval_py: dict_step_to_position_intervals[step][direction]) {
                map_step_to_position_intervals[step.cast<int>()][direction.cast<string>()].emplace_back(
                        make_shared<PositionInterval>(position_interval_py));
            }
        }
    }
    //for (auto const& [step_py, dict_position_intervals_py]:
    //        obj_semantic_model_py.attr("dict_step_to_position_intervals").cast<py::dict>()) {
    //    auto step = step_py.cast<int>();
    //    map_step_to_position_intervals[step] = map < string, vector < PositionIntervalPtr >> {};
    //
    //    for (auto const& [direction_py, list_position_interval_py]:
    //            dict_position_intervals_py.cast<py::dict>()) {
    //        auto direction = direction_py.cast<string>();
    //
    //        map_step_to_position_intervals[step][direction] = {};
    //
    //        for (auto const& position_interval_py: list_position_interval_py.cast<py::list>()) {
    //            map_step_to_position_intervals[step][direction].emplace_back(
    //                    make_shared<PositionInterval>(position_interval_py));
    //        }
    //    }
    //}
}