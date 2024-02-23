#pragma once

#include "reach_semantic/data_structure/proposition.hpp"
#include "reach_semantic/utility/pybind_includes.hpp"
#include "reach_semantic/utility/shared_include.hpp"

namespace semantic_reach {

/// Holder for propositions.
struct PropositionHolder {
    std::map<PropositionGroup, std::set<std::string>> map_group_to_set_propositions{};
    std::set<std::string> set_propositions{"true"};
    std::set<std::string> set_propositions_temporary{};

    PropositionHolder() = default;

    /// Constructor of PropositionHolder.
    /// @param set_propositions set of propositions
    /// @param group group of the propositions
    explicit PropositionHolder(std::set<std::string> const &set_propositions, PropositionGroup const &group);

    bool operator==(PropositionHolder const &other) const { return set_propositions == other.set_propositions; }

    struct HashFunction {
        size_t operator()(PropositionHolder const &proposition_holder) const {
            std::size_t hash = proposition_holder.set_propositions.size();

            for (auto const &proposition : proposition_holder.set_propositions) {
                boost::hash_combine(hash, proposition);
            }

            return hash;
        }
    };

    inline PropositionHolder clone() const {
        auto holder_cloned = PropositionHolder();

        holder_cloned.set_propositions = set_propositions;
        holder_cloned.set_propositions_temporary = set_propositions_temporary;
        holder_cloned.map_group_to_set_propositions = map_group_to_set_propositions;

        return holder_cloned;
    }

    inline void add_propositions(std::set<std::string> const &set_propositions, PropositionGroup const &group) {
        for (auto const &proposition : set_propositions) {
            add_proposition(proposition, group);
        }
    }

    inline void add_propositions(std::set<std::string> const &set_propositions, py::handle const &group_py) {
        for (auto const &proposition : set_propositions) {
            add_proposition(proposition, get_proposition_group(group_py.attr("name").cast<std::string>()));
        }
    }

    inline void add_proposition(std::string const &proposition, PropositionGroup const &group) {
        map_group_to_set_propositions[group].insert(proposition);

        if (group != PropositionGroup::TEMPORARY) {
            set_propositions.insert(proposition);
        } else {
            set_propositions_temporary.insert(proposition);
        }
    }

    inline std::set<std::string> propositions(bool include_temporary = true) const {
        if (include_temporary) {
            std::set<std::string> set_output{};

            std::set_union(set_propositions.begin(), set_propositions.end(), set_propositions_temporary.begin(),
                           set_propositions_temporary.end(), std::inserter(set_output, set_output.begin()));

            return set_output;

        } else {
            return set_propositions;
        }
    }

    inline std::set<std::string> propositions_in_group(PropositionGroup const &group) {
        return map_group_to_set_propositions[group];
    }

    inline std::set<std::string> propositions_in_group(py::handle const &group_py) {
        return map_group_to_set_propositions[get_proposition_group(group_py.attr("name").cast<std::string>())];
    }

    static inline PropositionGroup get_proposition_group(std::string const &group_name) {
        if (group_name == "POSITION")
            return PropositionGroup::POSITION;
        else if (group_name == "VELOCITY")
            return PropositionGroup::VELOCITY;
        else if (group_name == "ACCELERATION")
            return PropositionGroup::ACCELERATION;
        else if (group_name == "VEHICLE")
            return PropositionGroup::VEHICLE;
        else if (group_name == "TRAFFIC_SIGN")
            return PropositionGroup::TRAFFIC_SIGN;
        else if (group_name == "TRAFFIC_LIGHT")
            return PropositionGroup::TRAFFIC_LIGHT;
        else if (group_name == "INTERSECTION")
            return PropositionGroup::INTERSECTION;
        else if (group_name == "PRIORITY")
            return PropositionGroup::PRIORITY;
        else if (group_name == "TRAFFIC_STATUS")
            return PropositionGroup::TRAFFIC_STATUS;
        else if (group_name == "TEMPORARY")
            return PropositionGroup::TEMPORARY;
        else {
            std::cout << "No matching proposition group found." << std::endl;
            throw std::invalid_argument("No matching proposition group found.");
        }
    }
};

struct MultiStepPropositionHolder {
    std::set<std::string> set_propositions_time_invariant{};
    std::map<int, std::set<std::string>> map_step_to_set_propositions_time_variant{};

    std::map<PropositionGroup, std::set<std::string>> map_group_to_set_propositions_time_invariant{};
    std::map<PropositionGroup, std::map<int, std::set<std::string>>>
        map_group_to_step_to_set_propositions_time_variant{};

    MultiStepPropositionHolder() = default;

    ///// Constructor of PropositionHolder.
    ///// @param set_propositions set of propositions
    ///// @param group group of the propositions
    explicit MultiStepPropositionHolder(int const &step_end);

    explicit MultiStepPropositionHolder(py::handle const &obj_ph_py);

    inline auto time_invariant_propositions() const { return set_propositions_time_invariant; }

    inline auto time_invariant_propositions(PropositionGroup const &group) {
        return map_group_to_set_propositions_time_invariant[group];
    }

    inline auto time_variant_propositions() const { return map_step_to_set_propositions_time_variant; }

    inline auto time_variant_propositions(PropositionGroup const &group) {
        return map_group_to_step_to_set_propositions_time_variant[group];
    }

    inline auto time_variant_propositions_at_step(int const &step) {
        return map_step_to_set_propositions_time_variant[step];
    }

    inline auto time_variant_propositions_at_step(int const &step, PropositionGroup const &group) {
        return map_group_to_step_to_set_propositions_time_variant[group][step];
    }

    inline auto propositions_at_step(int const &step) {
        std::set<std::string> set_proposition{};

        std::set_union(set_propositions_time_invariant.begin(), set_propositions_time_invariant.end(),
                       map_step_to_set_propositions_time_variant[step].begin(),
                       map_step_to_set_propositions_time_variant[step].end(),
                       std::inserter(set_proposition, set_proposition.begin()));

        return set_proposition;
    }

    inline auto propositions_at_step(int const &step, PropositionGroup const &group) {
        std::set<std::string> set_proposition{};

        std::set_union(map_group_to_set_propositions_time_invariant[group].begin(),
                       map_group_to_set_propositions_time_invariant[group].end(),
                       map_group_to_step_to_set_propositions_time_variant[group][step].begin(),
                       map_group_to_step_to_set_propositions_time_variant[group][step].end(),
                       std::inserter(set_proposition, set_proposition.begin()));

        return set_proposition;
    }

    inline auto map_group_to_propositions_at_step(int const &step) {
        std::map<PropositionGroup, std::set<std::string>> map_group_to_set_propositions{};

        for (auto const &[group, set_propositions] : map_group_to_set_propositions_time_invariant) {
            map_group_to_set_propositions[group] = std::set<std::string>();
            for (auto const &proposition : set_propositions) {
                map_group_to_set_propositions[group].insert(proposition);
            }
        }

        for (auto &[group, map_step_to_set_propositions] : map_group_to_step_to_set_propositions_time_variant) {
            auto set_propositions = map_step_to_set_propositions[step];
            for (auto const &proposition : set_propositions) {
                map_group_to_set_propositions[group].insert(proposition);
            }
        }

        return map_group_to_set_propositions;
    }

    inline auto add_propositions(std::string const &proposition, PropositionGroup const &group, int const &step = -1) {
        if (step == -1) {
            set_propositions_time_invariant.insert(proposition);
            map_group_to_set_propositions_time_invariant[group].insert(proposition);

        } else {
            map_step_to_set_propositions_time_variant[step].insert(proposition);

            try {
                map_group_to_step_to_set_propositions_time_variant[group][step].insert(proposition);
            } catch (...) {
                map_group_to_step_to_set_propositions_time_variant[group][step] = std::set<std::string>{proposition};
            }
        }
    }

    static inline PropositionGroup get_proposition_group(std::string const &group_name) {
        if (group_name == "POSITION")
            return PropositionGroup::POSITION;
        else if (group_name == "VELOCITY")
            return PropositionGroup::VELOCITY;
        else if (group_name == "ACCELERATION")
            return PropositionGroup::ACCELERATION;
        else if (group_name == "VEHICLE")
            return PropositionGroup::VEHICLE;
        else if (group_name == "TRAFFIC_SIGN")
            return PropositionGroup::TRAFFIC_SIGN;
        else if (group_name == "TRAFFIC_LIGHT")
            return PropositionGroup::TRAFFIC_LIGHT;
        else if (group_name == "INTERSECTION")
            return PropositionGroup::INTERSECTION;
        else if (group_name == "PRIORITY")
            return PropositionGroup::PRIORITY;
        else if (group_name == "TRAFFIC_STATUS")
            return PropositionGroup::TRAFFIC_STATUS;
        else if (group_name == "TEMPORARY")
            return PropositionGroup::TEMPORARY;
        else {
            std::cout << "No matching proposition group found." << std::endl;
            throw std::invalid_argument("No matching proposition group found.");
        }
    }
};

using PropositionHolderPtr = std::shared_ptr<PropositionHolder>;
using MultiStepPropositionHolderPtr = std::shared_ptr<MultiStepPropositionHolder>;
} // namespace semantic_reach