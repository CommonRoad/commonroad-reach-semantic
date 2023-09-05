#include <iostream>
#include <chrono>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <yaml-cpp/yaml.h>

#include "geometry/curvilinear_coordinate_system.h"
#include "collision/collision_checker.h"
#include "reach_semantic/data_structure/reach/semantic_labeling_reach_set.hpp"
#include "reach_semantic/data_structure/reach/semantic_splitting_otf_reach_set.hpp"
#include "reach_semantic/data_structure/semantic_model.hpp"
#include "reachset/utility/collision_checker.hpp"
//#include "reach_semantic/semantic/data_structure/traffic_rule.hpp"

using namespace semantic_reach;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
namespace py = pybind11;

using CollisionCheckerPtr = collision::CollisionCheckerPtr;

int main() {
    // start the python interpreter and keep it alive
    py::scoped_interpreter python{};

    // ======== settings
    string path_root = "/home/lercher/tum/commonroad/commonroad-reach-semantic-addon/";
//    string name_scenario = "DEU_Test-1_1_T-1";
//    string name_scenario = "ZAM_Intersection-1_1_T-1";
//    string name_scenario = "ZAM_Merge-1_1_T-1";
    string name_scenario = "ESP_Monzon-2_2_T-1";

    // append path to interpreter
    py::module_ sys = py::module_::import("sys");
    sys.attr("path").attr("append")(path_root);
    sys.attr("path").attr("append")("/home/edmond/Softwares/commonroad/spot-cpp");

    // ======== configuration object via python ConfigurationBuilder
    auto cls_ConfigurationBuilder_py =
            py::module_::import("commonroad_reach_semantic.data_structure.config.semantic_configuration_builder").attr(
                    "SemanticConfigurationBuilder");
    auto obj_config_py = cls_ConfigurationBuilder_py.attr("build_configuration")(name_scenario, path_root);
    obj_config_py.attr("update")();
    obj_config_py.attr("print_configuration_summary")();

    auto config = obj_config_py.attr("convert_to_cpp_configuration")().cast<SemanticConfigurationPtr>();

    // ======== semantic model object via python SemanticModel
    auto cls_SemanticModel_py =
            py::module_::import("commonroad_reach_semantic.data_structure.environment_model.semantic_model").attr("SemanticModel");
    auto obj_semantic_model_py = cls_SemanticModel_py(obj_config_py);

    auto obj_dict_traffic_sign_to_priorities_py = py::module_::import("commonroad_reach_semantic.data_structure.rule.priorities").attr("dict_traffic_sign_to_priorities");

    obj_semantic_model_py.attr("determine_traffic_priorities")(obj_dict_traffic_sign_to_priorities_py);

    // ======== traffic rule interface object via python TrafficRuleInterface
    auto cls_TrafficRuleInterface_py = py::module_::import(
            "commonroad_reach_semantic.data_structure.rule.traffic_rule_interface").attr(
            "TrafficRuleInterface");
    auto obj_traffic_rule_interface_py = cls_TrafficRuleInterface_py(obj_config_py, obj_semantic_model_py);

    obj_traffic_rule_interface_py.attr("print_summary")();

    auto semantic_model = make_shared<SemanticModel>(obj_semantic_model_py);
    auto traffic_rule_interface = make_shared<TrafficRuleInterface>(obj_traffic_rule_interface_py);

    //// ======== CurvilinearCoordinateSystem
    //auto CLCS = make_shared<geometry::CurvilinearCoordinateSystem>(
    //        obj_config_py.attr("planning").attr("reference_path").cast<geometry::EigenPolyline>(),
    //        25.0, 0.1);
    //
    // ======== collision checker via python collision checker
    auto cls_CollisionChecker_py =
            py::module_::import("commonroad_reach.data_structure.collision_checker").attr("CollisionChecker");
    auto obj_collision_checker_py = cls_CollisionChecker_py(obj_config_py);
    auto collision_checker = obj_collision_checker_py.attr("cpp_collision_checker").cast<CollisionCheckerPtr>();

    // ======== ReachableSetInterface
//    auto reach_interface = SemanticLabelingReachableSet(config, collision_checker, semantic_model, traffic_rule_interface);
    auto reach_interface = SemanticSplittingOTFReachableSet(config, collision_checker, semantic_model, traffic_rule_interface);
    auto start = high_resolution_clock::now();
    reach_interface.compute();
    auto end = high_resolution_clock::now();
    cout << "Took: " << duration_cast<milliseconds>(end - start).count() << "ms" << endl;
    //
    //// ======== model checking with PyNuSMV
    //auto cls_ModelChecker_py =
    //        py::module_::import("commonroad_reachset.semantic.model_checker").attr("ModelChecker");
    //auto obj_model_checker_py = cls_ModelChecker_py(obj_config_py, reach_interface, obj_traffic_rule_py);
    //reach_interface = obj_model_checker_py.attr("check_model")().cast<ReachableSetInterface>();
    //
    //// ======== maneuver extraction
    //
    //// ======== visualization of results
    //if (visualize_results) {
    //    auto utils_visualization = py::module_::import("commonroad_reachset.common.utility.visualization");
    //    utils_visualization.attr("draw_scenario_with_reach_cpp")(obj_config_py, reach_interface,
    //                                                             py::arg("save_gif") = save_gif,
    //                                                             py::arg("save_fig") = save_fig,
    //                                                             py::arg("plot_refined") = plot_refined);
    //}
    cout << "Done." << endl;

    return 0;
}