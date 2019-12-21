# Temporary code while I wait for Carson and Aleem

import path_generator
import motion_profiler
import kinematics
import visualizer
import random
import time


def feet_to_meters(x):
    return x/3.28


TEST_POINTS = (
        (0, 0),
        (2, 0),
        (2, 200),  # TODO workaroud
)


def test_simulate_path():
    path = path_generator.generate_path_from_points(TEST_POINTS)
    motion_profile = motion_profiler.MotionProfile(path)

    DT = 0.1  # s
    RUNNING_TIME = 15  # s

    INITIAL_ROBOT_POSE = (TEST_POINTS[0][0], TEST_POINTS[0][1], 0)

    pose = INITIAL_ROBOT_POSE
    velocity = 0

    t = 0
    print()
    while True:  # t <= RUNNING_TIME:
        # The perturbation throws in some realism
        PERTURBATION_SIGMA = 0  # 0.01  # standard deviation
        pose = (
            pose[0] + random.normalvariate(0, PERTURBATION_SIGMA),
            pose[1] + random.normalvariate(0, PERTURBATION_SIGMA),
            pose[2] + random.normalvariate(0, PERTURBATION_SIGMA))
        # print(f'{t},{pose[0]},{pose[1]},{pose[2]}')
        visualizer.move(pose, path.last_lookahead_point)
        print(f'Distance on path: {path.distance_on_path}')
        drive_velocities = path.follow_path(pose, velocity, motion_profile)
        # print(f'{drive_velocities[0]}, {drive_velocities[1]}')
        pose = kinematics.find_new_pose(pose, drive_velocities, DT)
        velocity = (drive_velocities[0] + drive_velocities[1])/2
        t += DT
        # time.sleep(DT)

    # print(f'{t},{pose[0]},{pose[1]},{pose[2]}')
