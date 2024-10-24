#!/usr/bin/env python3
##
# @file dynamic_window_approach.py
#
# @brief Provide implementation of dynamic window approach controller for
# autonomous driving.
#
# @section author_doxygen_example Author(s)
# - Created by Dinh Ngoc Duc on 23/10/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import math
import numpy as np


class DynamicWindowApproach:
    """! Dynamic Window Approach controller
    The class provides implementation of dynamic window approach controller for
    autonomous driving.
    """
    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================

    def __init__(self, trajectory):
        """! Constructor
        @param trajectory<instance>: The trajectory
        """
        self.trajectory = trajectory

        self._register_model_parameters()

        self._register_dwa_parameters()

    def execute(self, state, input, index):
        """! Execute the controller
        @param state<list>: The state of the vehicle
        @param input<list>: The input of the vehicle
        @param previous_index<int>: The previous index
        @return<tuple>: The status and control
        """
        status = True

        if self._is_goal(state, self.trajectory):
            return False, [0, 0]

        goal = self._search_target_index(state)

        u = self._execute_dwa_control(state, goal, input)

        return status, u

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _register_model_parameters(self):
        """! Register the model parameters
        """
        self._min_speed = 0.0

        self._max_speed = 1.0

        self._delta_min = math.radians(-40.0)

        self._delta_max = math.radians(40.0)

        self._max_acceleration = 2.0

        self._max_delta_dot = math.radians(80)

        self._v_resolution = 0.1

        self._delta_resolution = math.radians(1.0)

    def _register_dwa_parameters(self):
        """! Register the DWA specification
        """
        self._dt = self.trajectory.sampling_time

        self._lookahead_steps = 2

        self._lookahead_time = self._lookahead_steps * self._dt

        self._to_goal_cost_gain = 1.0

        self._speed_cost_gain = 1.0

        self._tracking_cost_gain = [50.0, 50.0, 1.0, 20.0]

        self._goal_tolerance = 0.3

    def _find_nearest_index(self, state):
        """! Get the nearest waypoint from the current position
        @param state<list>: The state of the vehicle
        @return<list>: The index of the nearest waypoint
        """
        dx = [state[0] - ref_x for ref_x in self.trajectory.x[:, 0]]

        dy = [state[1] - ref_y for ref_y in self.trajectory.x[:, 1]]

        d = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]

        min_d = min(d)

        index = d.index(min_d)

        return index

    def _search_target_index(self, state):
        """! Get the tracking target index from the current position
        @param state<list>: The state of the vehicle
        @return<list>: The index of the tracking target
        """
        self.idx = self._find_nearest_index(state)

        goal = self.trajectory.x[self.idx]

        to_goal_distance = self._calculate_distance(state, goal)

        while to_goal_distance < self._goal_tolerance:

            if self.idx + 1 < len(self.trajectory.x):

                self.idx += 1

            else:
                break

            goal = self.trajectory.x[self.idx]

            to_goal_distance = self._calculate_distance(state, goal)

        if self.idx + self._lookahead_steps < len(self.trajectory.x):

            goal = self.trajectory.x[self.idx + self._lookahead_steps]

        else:

            goal = self.trajectory.x[-1]

        return goal

    def _execute_dwa_control(self, state, goal, input):
        """! Execute the DWA control
        @param state<list>: The state of the vehicle
        @param goal<list>: The goal of the vehicle
        @param input<list>: The input of the vehicle
        """
        dw = self._calculate_dynamic_window(input)

        [v, delta] = self._calculate_control(state, input, dw, goal)

        return [v, delta]

    def _calculate_dynamic_window(self, input):
        """! Calculate the dynamic window
        @note The dynamic window is the range of the vehicle's velocity
        and steering angle
        @param input<list>: The input of the vehicle
        @return<list>: The dynamic window of the vehicle
        """
        vs = [self._min_speed, self._max_speed,
              self._delta_min, self._delta_max]

        vd = [input[0] - self._max_acceleration * self._dt,
              input[0] + self._max_acceleration * self._dt,
              input[1] - self._max_delta_dot * self._dt,
              input[1] + self._max_delta_dot * self._dt]

        dynamic_window = [max(vs[0], vd[0]),
                          min(vs[1], vd[1]),
                          max(vs[2], vd[2]),
                          min(vs[3], vd[3])]

        return dynamic_window

    def _calculate_control(self, state, input, dw, goal):
        """! Calculate the best control input
        The best control of all sample is the one that minimize the cost
        @note Cost = to_goal_cost + speed_cost
        @note to_goal_cost = distance to goal
        @note speed_cost = difference between current speed and max speed
        
        @param state<list>: The state of the vehicle
        @param input<list>: The input of the vehicle
        @param dw<list>: The dynamic window
        @param goal<list>: The goal of the vehicle
        @return<list>: The best control input to reach the goal
        """
        min_cost = float("inf")

        best_v, best_delta = 0.0, 0.0

        for v in np.arange(dw[0], dw[1], self._v_resolution):
            for delta in np.arange(dw[2], dw[3], self._delta_resolution):

                lookahead_trajectory = self._predict_trajectory(
                    state, [v, delta])

                to_goal_cost = self._to_goal_cost_gain * \
                    self._calculate_to_goal_cost(lookahead_trajectory, goal)

                speed_cost = self._speed_cost_gain * \
                    self._calculate_speed_cost([v, delta])

                final_cost = to_goal_cost + speed_cost

                if min_cost >= final_cost:

                    min_cost = final_cost

                    best_v = v

                    best_delta = delta

        return [best_v, best_delta]

    def _predict_trajectory(self, state, input):
        """! Predict the trajectory using bicycle model
        @param state<list>: The state of the vehicle
        @param input<list>: The input of the vehicle
        """
        trajectory = np.array(state)

        time = 0

        while time <= self._lookahead_time:

            state = self._update_state(state, input, self._dt)

            trajectory = np.vstack((trajectory, state))

            time += self._dt

        return trajectory

    def _calculate_to_goal_cost(self, predict_trajectory, goal):
        """! Calculate the cost to the goal
        @param trajectory<list>: The trajectory
        @param goal<list>: The goal
        @return<float>: The cost to the goal
        """
        dx = goal[0] - predict_trajectory[-1, 0]

        dy = goal[1] - predict_trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - predict_trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        return cost

    def _calculate_speed_cost(self, input):
        """! Calculate the cost of the velocity
        @param input<list>: The input of the vehicle
        """
        speed_cost = (self._max_speed - input[0])

        return speed_cost

    # ==================================================================================================
    # STATIC METHODS
    # ==================================================================================================
    @staticmethod
    def _is_goal(state, trajectory):
        """! Check if the vehicle has reached the goal
        @param state<list>: The state of the vehicle
        @param trajectory<instance>: The trajectory
        @return<bool>: The flag to indicate if the vehicle has reached the goal
        """
        delta_x = trajectory.x[-1, 0] - state[0]

        delta_y = trajectory.x[-1, 1] - state[1]

        distance = np.hypot(delta_x, delta_y)

        return distance < 0.1

    @staticmethod
    def _calculate_distance(current_x, reference_x):
        """! Calculate the distance function
        @param current_x<list>: The current position
        @param reference_x<list>: The reference position
        @return<list>: The distance between the current position
        and the reference position
        """
        distance = current_x - reference_x

        x = distance[:, 0] if distance.ndim == 2 else distance[0]

        y = distance[:, 1] if distance.ndim == 2 else distance[1]

        return np.hypot(x, y)

    @staticmethod
    def _update_state(state, input, dt):
        """! Compute the next state
        @param state<list>: The current state of the vehicle
        @param input<list>: The input of the vehicle
        @param dt<float>: The time step
        @return next_state<list>: The next state of the vehicle
        """
        length_base = 1.0

        dfdt = np.array([input[0] * math.cos(input[1]) * math.cos(state[2]),
                         input[0] * math.cos(input[1]) * math.sin(state[2]),
                        (input[0] / length_base) * math.sin(input[1])])

        next_state = state + dfdt * dt

        return next_state
