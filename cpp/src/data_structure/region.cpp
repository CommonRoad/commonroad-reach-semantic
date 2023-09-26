#include "reach_semantic/data_structure/region.hpp"
#include "reachset/utility/shared_using.hpp"

using namespace semantic_reach;

Region::Region(pybind11::handle const &obj_region_py) {
    step_end = obj_region_py.attr("step_end").cast<int>();

    set_ids_lanelets = obj_region_py.attr("set_ids_lanelets").cast<set<int>>();

    auto vec_vertices_cart = obj_region_py.attr("polygon_cart").attr("vertices").cast<vector<tuple<double, double>>>();
    polygon_cart = make_shared<reach::ReachPolygon>(vec_vertices_cart);

    auto vec_vertices_cvln = obj_region_py.attr("polygon_cvln").attr("vertices").cast<vector<tuple<double, double>>>();
    polygon_cvln = make_shared<reach::ReachPolygon>(vec_vertices_cvln);

    proposition_holder = make_shared<MultiStepPropositionHolder>(obj_region_py.attr("proposition_holder"));
}

bool Region::intersects(reach::ReachPolygonPtr const &rectangle, string const &coordinate_system) const {
    auto [p_lon_min_box, p_lat_min_box, p_lon_max_box, p_lat_max_box] = rectangle->bounding_box();

    double p_lon_min, p_lat_min, p_lon_max, p_lat_max;
    if (coordinate_system == "CART")
        std::tie(p_lon_min, p_lat_min, p_lon_max, p_lat_max) = polygon_cart->bounding_box();

    else if (coordinate_system == "CVLN")
        std::tie(p_lon_min, p_lat_min, p_lon_max, p_lat_max) = polygon_cvln->bounding_box();

    else
        throw std::logic_error("<Region> Provided coordinate system is invalid.");

    return p_lon_max_box >= p_lon_min && p_lon_min_box <= p_lon_max && p_lat_max_box >= p_lat_min &&
        p_lat_min_box <= p_lat_max;
}
