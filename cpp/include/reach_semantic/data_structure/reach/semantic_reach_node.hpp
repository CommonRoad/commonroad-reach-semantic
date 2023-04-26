#pragma once

#include "reach_semantic/utility/shared_include.hpp"
#include "reach_semantic/data_structure/proposition_holder.hpp"
#include "reach_semantic/data_structure/reach/reach_polygon_boost.hpp"

namespace reach {
/// Node within the reachability graph, also used in the reachable set computation.
/// Each node is a Cartesian product of polygon_lon and polygon_lat. In curvilinear coordinate system, polygon_lon
/// is a polygon in the longitudinal p-v domain, and polygon_lat is a polygon in the lateral p-v domain; In Cartesian
/// coordinate system, they represent polygons in the x-v and y-v domains, respectively. SemanticReachNode inherits ReachBaseSet.
/// In addition, it has a step, an ID, and lists of parent nodes and child nodes.
class SemanticReachNode {
public:
    SemanticReachNode() = default;

    /// Constructor of SemanticReachNode.
    /// @param step step of the node
    /// @param polygon_lon longitudinal polygon of the node
    /// @param polygon_lat lateral polygon of the node
    /// @param proposition_holder holder for propositions
    /// @param source_propagation source nodes of the propagation
    SemanticReachNode(int const& step, ReachPolygonPtr polygon_lon, ReachPolygonPtr polygon_lat,
              PropositionHolder proposition_holder,
              std::vector<std::shared_ptr<SemanticReachNode>> const& source_propagation = {});

    static int cnt_id;
    int id{};
    int step{};
    ReachPolygonPtr polygon_lon;
    ReachPolygonPtr polygon_lat;
    std::vector<std::shared_ptr<SemanticReachNode>> vec_nodes_parent;
    std::vector<std::shared_ptr<SemanticReachNode>> vec_nodes_child;
    std::vector<std::shared_ptr<SemanticReachNode>> vec_nodes_source;
    // the vector of nodes from which the current node is originated
    PropositionHolder proposition_holder;
    std::set<int> set_ids_lanelets{};


    inline std::shared_ptr<SemanticReachNode> clone() const {
        auto node_clone =
                std::make_shared<SemanticReachNode>(step,
                                            polygon_lon->clone(),
                                            polygon_lat->clone(),
                                            proposition_holder.clone(),
                                            vec_nodes_source);
        node_clone->vec_nodes_child = vec_nodes_child;
        node_clone->vec_nodes_parent = vec_nodes_parent;
        node_clone->set_ids_lanelets = set_ids_lanelets;

        return node_clone;
    }

    inline std::tuple<double, double, double, double> box_lon() const { return polygon_lon->bounding_box(); }

    inline std::tuple<double, double, double, double> box_lat() const { return polygon_lat->bounding_box(); }

    inline double p_lon_min() const { return std::get<0>(this->box_lon()); }

    inline double p_lon_max() const { return std::get<2>(this->box_lon()); }

    inline double v_lon_min() const { return std::get<1>(this->box_lon()); }

    inline double v_lon_max() const { return std::get<3>(this->box_lon()); }

    inline double p_lat_min() const { return std::get<0>(this->box_lat()); }

    inline double p_lat_max() const { return std::get<2>(this->box_lat()); }

    inline double v_lat_min() const { return std::get<1>(this->box_lat()); }

    inline double v_lat_max() const { return std::get<3>(this->box_lat()); }

    inline std::set<std::string> set_propositions(bool include_temporary = true) const {
        return proposition_holder.propositions(include_temporary);
    }

    /// Rectangle representing the projection of the node onto the position domain.
    inline ReachPolygonPtr position_rectangle() const {
        return std::make_shared<ReachPolygon>(p_lon_min(), p_lat_min(),
                                              p_lon_max(), p_lat_max());
    }

    inline void assign_parent_nodes(std::vector<std::shared_ptr<SemanticReachNode>> const& vec_nodes_parent) {
        this->vec_nodes_parent = vec_nodes_parent;
    }

    void assign_child_nodes(std::vector<std::shared_ptr<SemanticReachNode>> const& vec_nodes_child) {
        this->vec_nodes_child = vec_nodes_child;
    }

    inline void clear_parent_nodes() {
        vec_nodes_parent.clear();
    }

    inline void clear_child_nodes() {
        vec_nodes_child.clear();
    }

    /// Increases the ID of the node by 1.
    inline void increment_id() {
        id = SemanticReachNode::cnt_id++;
    }

    inline void add_lanelet_ids(std::set<int> const& set_ids_lanelets) {
        this->set_ids_lanelets.insert(set_ids_lanelets.begin(), set_ids_lanelets.end());
    }

    /// Intersects with the given rectangle in position domain.
    inline void intersect_in_position_domain(double const& p_lon_min, double const& p_lat_min,
                                             double const& p_lon_max, double const& p_lat_max) {
        polygon_lon->intersect_halfspace(1, 0, p_lon_max);
        polygon_lon->intersect_halfspace(-1, 0, -p_lon_min);
        polygon_lat->intersect_halfspace(1, 0, p_lat_max);
        polygon_lat->intersect_halfspace(-1, 0, -p_lat_min);
    }

    /// Intersects with the given velocities in velocity domain.
    inline void intersect_in_velocity_domain(double const& v_lon_min, double const& v_lat_min,
                                             double const& v_lon_max, double const& v_lat_max) {
        polygon_lon->intersect_halfspace(0, 1, v_lon_max);
        polygon_lon->intersect_halfspace(0, -1, -v_lon_min);
        polygon_lat->intersect_halfspace(0, 1, v_lat_max);
        polygon_lat->intersect_halfspace(0, -1, -v_lat_min);
    }

    bool add_parent_node(std::shared_ptr<SemanticReachNode> const& node_parent);

    bool remove_parent_node(std::shared_ptr<SemanticReachNode> const& node_parent);

    bool add_child_node(std::shared_ptr<SemanticReachNode> const& node_child);

    bool remove_child_node(std::shared_ptr<SemanticReachNode> const& node_child);

    /// Resets the ID counter of nodes to zero.
    inline static void reset_id_counter() {
        SemanticReachNode::cnt_id = 0;
    }
};

using SemanticReachNodePtr = std::shared_ptr<SemanticReachNode>;
}