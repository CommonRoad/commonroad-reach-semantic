from typing import Set

from commonroad_reach_semantic.data_structure.environment_model.vehicle import Vehicle
from commonroad_reach_semantic.data_structure.rule.proposition import Proposition as P


class PositionInterval:
    """Class to represent position intervals in which a set of propositions hold."""

    def __init__(self, p_min, p_max, set_propositions: Set):
        self.p_min: float = p_min
        self.p_max: float = p_max
        self.set_propositions: Set = set_propositions.copy()

    def __repr__(self):
        return f"Interval({self.p_min:.3f},{self.p_max:.3f},{self.set_propositions})"

    def clone(self):
        return PositionInterval(self.p_min, self.p_max, self.set_propositions)

    def intersects(self, p_min, p_max):
        if p_min > self.p_max or p_max < self.p_min:
            return False

        return True

    def split_with_respect_to_vehicle(self, step: int, vehicle: Vehicle, dimension_ego: float, direction: str):
        """Returns a list of longitudinal position intervals for the input vehicle

        An interval is split into multiple intervals if it overlaps with the vehicle.
        """
        if direction == "lon":
            return self._split_with_respect_to_vehicle_in_longitudinal_direction(step, vehicle, dimension_ego)

        elif direction == "lat":
            return self._split_with_respect_to_vehicle_in_lateral_direction(step, vehicle, dimension_ego)

        else:
            raise Exception("Given direction is not valid.")

    def _split_with_respect_to_vehicle_in_longitudinal_direction(self, step: int, vehicle: Vehicle,
                                                                 half_length_ego: float):
        """Splits position interval in longitudinal direction."""
        try:
            p_lon_min_ref = vehicle.p_lon_min_ref(step, half_length_ego)
            p_lon_max_ref = vehicle.p_lon_max_ref(step, half_length_ego)

        # vehicle is out of projection domain
        except (KeyError, AttributeError, TypeError):
            return [self]

        list_intervals = []
        id_vehicle = vehicle.id_vehicle
        # interval completely in front of vehicle
        if self.p_min >= p_lon_max_ref:
            self.set_propositions.add(P.in_front_of(id_vehicle))
            list_intervals.append(self)

        # interval completely behind vehicle
        elif self.p_max <= p_lon_min_ref:
            self.set_propositions.add(P.behind(id_vehicle))
            list_intervals.append(self)

        # interval encloses vehicle |---(   )---|, split into three intervals
        elif self.p_min < p_lon_min_ref < p_lon_max_ref < self.p_max:
            list_intervals.append(PositionInterval(self.p_min,
                                                   p_lon_min_ref,
                                                   self.set_propositions.union({P.behind(id_vehicle)})))

            list_intervals.append(PositionInterval(p_lon_min_ref,
                                                   p_lon_max_ref,
                                                   self.set_propositions.union({P.beside(id_vehicle)})))

            list_intervals.append(PositionInterval(p_lon_max_ref,
                                                   self.p_max,
                                                   self.set_propositions.union({P.in_front_of(id_vehicle)})))

        # vehicle encloses interval
        elif p_lon_min_ref < self.p_min < self.p_max < p_lon_max_ref:
            self.set_propositions.add(P.beside(id_vehicle))
            list_intervals.append(self)

        # |---( | ), overlaps at the front side, split into two intervals
        elif self.p_min < p_lon_min_ref < self.p_max <= p_lon_max_ref:
            list_intervals.append(PositionInterval(self.p_min,
                                                   p_lon_min_ref,
                                                   self.set_propositions.union({P.behind(id_vehicle)})))

            list_intervals.append(PositionInterval(p_lon_min_ref,
                                                   self.p_max,
                                                   self.set_propositions.union({P.beside(id_vehicle)})))

        # ( | )-----|, overlaps at the back side, split into two intervals
        elif p_lon_min_ref <= self.p_min < p_lon_max_ref < self.p_max:
            list_intervals.append(PositionInterval(self.p_min,
                                                   p_lon_max_ref,
                                                   self.set_propositions.union({P.beside(id_vehicle)})))
            list_intervals.append(PositionInterval(p_lon_max_ref,
                                                   self.p_max,
                                                   self.set_propositions.union({P.in_front_of(id_vehicle)})))

        return list_intervals

    def _split_with_respect_to_vehicle_in_lateral_direction(self, step: int, vehicle: Vehicle,
                                                            half_width_ego: float):
        """Splits position interval in lateral direction."""
        # vehicle is out of projection domain
        try:
            p_lat_min_ref = vehicle.p_lat_min_ref(step, half_width_ego)
            p_lat_max_ref = vehicle.p_lat_max_ref(step, half_width_ego)

        except (KeyError, AttributeError, TypeError):
            return [self]

        list_intervals = []
        id_vehicle = vehicle.id_vehicle
        # interval is completely left of vehicle
        if self.p_min >= p_lat_max_ref:
            self.set_propositions.add(P.left_of(id_vehicle))
            list_intervals.append(self)

        # interval is completely right of vehicle
        elif self.p_max <= p_lat_min_ref:
            self.set_propositions.add(P.right_of(id_vehicle))
            list_intervals.append(self)

        # interval encloses vehicle |---(   )---|
        elif self.p_min < p_lat_min_ref < p_lat_max_ref < self.p_max:
            list_intervals.append(PositionInterval(self.p_min,
                                                   p_lat_min_ref,
                                                   self.set_propositions.union({P.right_of(id_vehicle)})))
            list_intervals.append(PositionInterval(p_lat_min_ref,
                                                   p_lat_max_ref,
                                                   self.set_propositions.union({P.aligned_with(id_vehicle)})))
            list_intervals.append(PositionInterval(p_lat_max_ref,
                                                   self.p_max,
                                                   self.set_propositions.union({P.left_of(id_vehicle)})))

        # vehicle encloses interval
        elif p_lat_min_ref < self.p_min < self.p_max < p_lat_max_ref:
            self.set_propositions.add(P.aligned_with(id_vehicle))
            list_intervals.append(self)

        # |---( | )
        elif self.p_min < p_lat_min_ref < self.p_max <= p_lat_max_ref:
            list_intervals.append(PositionInterval(self.p_min,
                                                   p_lat_min_ref,
                                                   self.set_propositions.union({P.right_of(id_vehicle)})))

            list_intervals.append(PositionInterval(p_lat_min_ref,
                                                   self.p_max,
                                                   self.set_propositions.union({P.aligned_with(id_vehicle)})))

        # ( | )---|
        elif p_lat_min_ref <= self.p_min < p_lat_max_ref < self.p_max:
            list_intervals.append(PositionInterval(self.p_min,
                                                   p_lat_max_ref,
                                                   self.set_propositions.union({P.aligned_with(id_vehicle)})))

            list_intervals.append(PositionInterval(p_lat_max_ref,
                                                   self.p_max,
                                                   self.set_propositions.union({P.left_of(id_vehicle)})))

        return list_intervals
