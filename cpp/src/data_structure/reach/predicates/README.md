### Adding new predicates

1. Create a subclass of `CppPredicate` in the directory corresponding to the predicate's type (e.g. `position` for
   predicates related to the position of the ego vehicle).
2. Implement the methods `_restrict_reach_node_mandatory` and `_restrict_reach_node_forbidden`
    * `_restrict_reach_node_mandatory` returns new reach nodes that (tightly) overapproximate the part of the original
      reach node that satisfies the predicate.
    * `_restrict_reach_node_forbidden` returns new reach nodes that (tightly) overapproximate the part of the original
      reach node that does not satisfy the predicate.
3. Register the new predicate in `PredicateFactory::predicate_from_proposition` and create a new method in the factory
   for creating the predicate (`PredicateFactory::make_xxx_predicate`).

Do not forget to write a test for the new predicate.
