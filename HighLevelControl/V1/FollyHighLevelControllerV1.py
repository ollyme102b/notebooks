import numpy as np
from MPC_Alg.v1.CFTOCSolverV1 import CFTOCSolverV1
from Path_Planning.level1.circleLineIntersection import circle_line_intersection


class FollyHighLevelControllerV1:
    """
    Folly High Level Controller Version 1
    """

    def __init__(self, molly_initial_position,
                 folly_initial_position,
                 object_length,
                 line_path_start_point,
                 line_path_end_point,
                 horizon,
                 step_time,
                 max_speed):
        """
        Initialized controller
        :param molly_initial_position: molly initial position
        :param folly_initial_position: folly initial position
        :param object_length: lifted object length
        :param line_path_start_point: Folly path line constraint start point
        :param line_path_end_point: Folly path line constraint end point
        :param horizon: MPC horizon
        :param step_time: time in between MPC steps
        :param max_speed: maximum allowable Folly speed
        """
        self._object_length = object_length
        self._line_path_start_point = line_path_start_point
        self._line_path_end_point = line_path_end_point

        folly_desired_position = self._desired_position(molly_initial_position, folly_initial_position)

        A = np.eye(2)  # state dynamics
        B = step_time * np.eye(2)  # input velocity dynamics

        self.optimizer = CFTOCSolverV1(A, B, folly_initial_position, folly_desired_position, horizon, max_speed)

    def _desired_position(self, molly_position, folly_position):
        """
        computes Folly desired position analytically as interstection of circle and line
        :param molly_position: current molly position as center of circle
        :param folly_position: current folly position to deduce better of two circle intersect line solutions
        :return: folly desired position
        """
        folly_desired_position_x, folly_desired_position_y = circle_line_intersection(molly_position[0],
                                                                                      molly_position[1],
                                                                                      self._object_length,
                                                                                      self._line_path_start_point[0],
                                                                                      self._line_path_start_point[1],
                                                                                      self._line_path_end_point[0],
                                                                                      self._line_path_end_point[1],
                                                                                      xp=folly_position[0],
                                                                                      yp=folly_position[1]
                                                                                      )
        return np.array([folly_desired_position_x, folly_desired_position_y])

    def optimal_input(self, current_molly_position, current_folly_position):
        """
        calculates optimal input for controller
        :param current_molly_position: current molly position
        :param current_folly_position: current folly position
        :return: optimal velocity command
        """
        folly_desired_position = self._desired_position(current_molly_position, current_folly_position)
        return self.optimizer.calculate_optimal_actuation(current_folly_position, folly_desired_position)
