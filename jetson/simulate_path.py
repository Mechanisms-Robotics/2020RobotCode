# Temporary code while I wait for Carson and Aleem

import path_generator
import motion_profiler
import kinematics
import visualizer
import random


TEST_POINTS = (
    (0, 0),
    (3, -2),
    (7, -2),
    (7, 2),
    (3, 2),
)


def test_simulate_path():
    path = path_generator.generate_path_from_points(TEST_POINTS)
    motion_profile = motion_profiler.MotionProfile(path)

    DT = 0.02  # s
    RUNNING_TIME = 15  # s

    INITIAL_ROBOT_POSE = (TEST_POINTS[0][0], TEST_POINTS[0][1], 0)

    pose = INITIAL_ROBOT_POSE
    velocity = 0

    t = 0
    print()
    while t <= RUNNING_TIME:
        # The perturbation throws in some realism
        PERTURBATION_SIGMA = 0.01  # standard deviation
        pose = (
            pose[0] + random.normalvariate(0, PERTURBATION_SIGMA),
            pose[1] + random.normalvariate(0, PERTURBATION_SIGMA),
            pose[2] + random.normalvariate(0, PERTURBATION_SIGMA))
        # print(f'{t},{pose[0]},{pose[1]},{pose[2]}')
        visualizer.update(pose)
        drive_velocities = path.follow_path(pose, velocity, motion_profile)
        print(f'{drive_velocities[0]}, {drive_velocities[1]}')
        pose = kinematics.find_new_pose(pose, drive_velocities, DT)
        velocity = (drive_velocities[0] + drive_velocities[1])/2  # I think...
        t += DT

    # print(f'{t},{pose[0]},{pose[1]},{pose[2]}')
