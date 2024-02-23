#pragma once

#include <boost/functional/hash.hpp>

#include <unordered_map>
#include <unordered_set>

namespace semantic_reach {
using Literal = std::pair<std::string, bool>;
using LiteralSet = std::unordered_set<Literal>;
using Minterm = LiteralSet;
template <typename T> using MintermMap = std::unordered_map<Minterm, T>;
} // namespace semantic_reach

namespace std {
template <> struct hash<semantic_reach::Literal> {
    size_t operator()(const semantic_reach::Literal &literal) const { return boost::hash_value(literal); }
};

template <> struct hash<semantic_reach::LiteralSet> {
    size_t operator()(const semantic_reach::LiteralSet &literal_set) const {
        size_t seed = literal_set.size();
        for (const auto &literal : literal_set) {
            seed ^= boost::hash_value(literal);
        }
        return seed;
    }
};
} // namespace std
