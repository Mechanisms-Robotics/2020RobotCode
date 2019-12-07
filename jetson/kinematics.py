import math
import utils


class RobotModel:
    WHEEL_BASE = 1.5  # m  (tuned)
    MAX_VELOCITY = 3.5  # m / s  (3 m/s is about 10 ft/s)
    MIN_VELOCITY = 0.1


# thanks http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf


MIN_TURN_RADIUS = 0.00001  # m  See comment below about possibly tuning this


def find_drive_velocities(velocity, turn_radius):
    '''
    Given a desired translational velocity and a desired turn radius, return
    the differential drive velocities required to obtain those parameters.

    This is inverse kinematics.  What outputs yield the desired motion?
    '''

    # Observe that if turn_radius == float('inf'), vr = vl = velocity.  If
    # velocity == 0, vr = vl = 0.  If turn_radius == 0, vr = -vl (and I guess
    # that any velocity > 0 leads to a collapse of the spacetime continuum).

    # According to the handy paper above from the smart folks at Columbia,
    # Vr = w(R + l/2) and Vl = w(R - l/2) where w is omega and l is wheel base.
    # I can use my handy physics degree to realize that w = v/R (radians per
    # second).  This concords with Magnus's lectures and Carson's solution.

    if abs(turn_radius) < MIN_TURN_RADIUS:
        # This idea may be ill conceived or overthinking things, but it at
        # least prevents a divide by zero and could be tuned to limit overly
        # aggressive turns.
        turn_radius = math.copysign(MIN_TURN_RADIUS, turn_radius)

    omega = velocity / turn_radius
    vl = omega * (turn_radius - RobotModel.WHEEL_BASE/2)
    vr = omega * (turn_radius + RobotModel.WHEEL_BASE/2)

    return (vl, vr)


def test_find_drive_velocities():
    pass
    # TODO  Keep adding tests / incorporate Carson and Aleem's code for this


def find_new_pose(pose, drive_velocities, dt):
    '''
    Given a pose (x, y, theta) and velocities (vl, vr), return a new robot
    pose.
    '''

    # based on Magnus lecture

    x, y, theta = pose
    dl = drive_velocities[0]*dt
    dr = drive_velocities[1]*dt
    dc = (dl + dr)/2
    xp = x + dc*math.cos(theta)
    yp = y + dc*math.sin(theta)
    thetap = theta + (dr - dl)/RobotModel.WHEEL_BASE
    return (xp, yp, thetap)

    # based on Columbia paper

    x = pose[0]
    y = pose[1]
    theta = pose[2]
    (vl, vr) = drive_velocities

    if vl == vr:
        d = vl*dt
        return (x+d*math.cos(theta), y+d*math.sin(theta), theta)

    R = RobotModel.WHEEL_BASE/2*(vl + vr)/(vr - vl)
    omega = (vr - vl)/RobotModel.WHEEL_BASE

    ICCx = x - R*math.sin(theta)
    ICCy = y + R*math.cos(theta)

    dx = x - ICCx
    dy = y - ICCy

    omega_dt = omega*dt
    cos_omega_dt = math.cos(omega_dt)
    sin_omega_dt = math.sin(omega_dt)

    xp = cos_omega_dt*dx - sin_omega_dt*dy + ICCx
    yp = sin_omega_dt*dx + cos_omega_dt*dy + ICCy
    thetap = theta + omega_dt

    return (xp, yp, thetap)


def test_find_new_pose():
    INITIAL_POSE = (0, 0, math.pi/4)
    VELOCITIES = (1, 1)  # test straight case
    EXPECTED_X = math.sqrt(2)/2
    EXPECTED_Y = math.sqrt(2)/2
    EXPECTED_THETA = math.pi/4
    pose = find_new_pose(INITIAL_POSE, VELOCITIES, 1)
    utils.assertNear(pose[0], EXPECTED_X)
    utils.assertNear(pose[1], EXPECTED_Y)
    utils.assertNear(pose[2], EXPECTED_THETA)

    # TODO more tests


###############################################################################
#
# CARSON CODE TO INTEGRATE AND TEST  TODO
#
###############################################################################

# FORWARD KINEMATICS - How will these inputs move me?

def next_pose(turn_radius, pose, left_vel, right_vel, t):
    center_v = (left_vel + right_vel)/2  # simpler way to do the above code
    # full circle degrees/time to finish circumference given robot velocity
    angular_v = center_v/turn_radius
    delta_theta = angular_v*t  # radians robot travels on circular path

    new_theta = delta_theta+pose[2]  # new theta based on old theta and change
    delta_x = math.cos(new_theta)-math.cos(pose[2])   # change in x
    delta_y = math.sin(new_theta)-math.sin(pose[2])   # change in y
    new_x = pose[0]+delta_x   # new x based on old x and change
    new_y = pose[1]+delta_y   # new y based on old y and change

    return((new_x, new_y, new_theta))


# INVERSE KINEMATICS - What outputs will move me there?

def carson_find_velocities(turn_radius, velocity):  # Carson's solution
    # TODO: test and incorporate (this is really more for the simulator)
    wheel_distance = RobotModel.WHEEL_BASE

    angular_velocity = velocity/turn_radius   # find angular velocity

    left_radius = turn_radius-(wheel_distance/2)
    right_radius = turn_radius+(wheel_distance/2)

    if (turn_radius < 0):  # if the radius is negative, it's a right turn
        left_radius, right_radius = right_radius, left_radius  # swap values

    # angular velocity is constant, multiply by radius to get left velocity
    left_velocity = left_radius*angular_velocity
    right_velocity = right_radius*angular_velocity    # same for right

    return((left_velocity, right_velocity))


def test_compare_carson_and_mr_odom():
    # Simply compare our solutions to sanity check them
    TURN_RADIUS = 30  # m
    VELOCITY = 3  # m / s

    carson = carson_find_velocities(TURN_RADIUS, VELOCITY)
    mr_odom = find_drive_velocities(VELOCITY, TURN_RADIUS)
    assert carson == mr_odom
