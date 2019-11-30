'''
This module contains code to govern motion of the robot along a path.
'''

import kinematics
import path_follower
import logging


class MotionProfile:
    def __init__(self, path):
        self.path = path
        self.path_length = path.get_length()

    def get_desired_velocity(self, distance_on_path, current_velocity):
        '''
        Given the distance along the path (meters), return the desired
        velocity (m/s).

        TODO: This is just an experimental implementation.  Why do we have
        distance_on_path here when I can get it from self.path?
        '''

        logging.debug(
            'Motion profiler distance on path: %s' % distance_on_path)

        RAMP_UP_DISTANCE = 2.0  # m
        ramp_up = min(1.0, distance_on_path/RAMP_UP_DISTANCE)
        ramp_down = min(1.0,
            (self.path_length - distance_on_path)/RAMP_UP_DISTANCE)
        desired_velocity = kinematics.RobotModel.MAX_VELOCITY*ramp_up*ramp_down

#        if desired_velocity == current_velocity:
#            return current_velocity
#
#        RAMP_RATE = 0.1  # TODO: just for experimenting
#
#        if desired_velocity > current_velocity:
#            desired_velocity = current_velocity + RAMP_RATE
#        else:
#            desired_velocity = current_velocity - RAMP_RATE

        return max(desired_velocity, kinematics.RobotModel.MIN_VELOCITY)


def test_get_desired_velocity():
    path = path_follower.LinearPathSegment((0, 0), (10, 10))
    motion_profile = MotionProfile(path)
    motion_profile.get_desired_velocity(0)
    # TODO under construction
