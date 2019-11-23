'''
This module contains code to govern motion of the robot along a path.
'''

import kinematics
import path_follower


class MotionProfile:
    def __init__(self, path):
        self.path = path

    def get_desired_velocity(self, distance_on_path):
        '''
        Given the distance along the path (meters), return the desired
        velocity (m/s).
        '''

        return kinematics.RobotModel.MAX_VELOCITY  # TODO


def test_get_desired_velocity():
    path = path_follower.LinearPathSegment((0, 0), (10, 10))
    motion_profile = MotionProfile(path)
    motion_profile.get_desired_velocity(0)
    # TODO under construction
