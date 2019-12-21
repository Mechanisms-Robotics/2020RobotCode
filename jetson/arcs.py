'''
This is a module to work with circular arcs.  The code is by Alex and Carson.
The idea is to take a robot pose, (x, y, theta), and a point, (x, y), and
return an arc that will connect the robot to the point.  The arc projects from
the nose of the robot and passes through the point.  A negative radius
indicates that the robot should turn right to follow the arc, a radius of
infinity indicates that the robot should drive directly forward, and a radius
of negative infinity indicates that the robot should go directly backwards.
An arc radius of zero is possible since this is what we will return if the
robot is exactly at the goal.  We use float('inf') if the robot should drive
straight forward, and float('-inf') if the point is directly behind the robot.

Integrated, tested and approved by Mr. Odom of 4910.
'''

import random  # used for tests
import utils

def current_solution(pose, point):
    return carson_solution(pose, point)
    # return alex_solution(pose, point)

###############################################################################
#
# BEGIN CARSON'S CODE (which may be formatted to fit your screen)
#
###############################################################################

def carson_solution(pose, point):
  return pure_pursuit(pose[0], pose[1], pose[2], point[0], point[1])

#Code by Carson Duffy
#Property of FRC Team 4910: East Cobb Robotics

import math
import numpy as np

def pure_pursuit(x, y, theta, target_x, target_y):
    center_v=(-math.sin(theta), math.cos(theta))    #perpendicular vector to robot (center seeking), assume it is a left turn
    dist_v=(target_x-x, target_y-y)                 #distance from the current point to the target point as a vector

    reflect_v=reflect(dist_v, center_v)     #reflected dist_v across center_v
    point=(x+reflect_v[0], y+reflect_v[1])  #point on the circle created by (x, y) and reflect_v3

    center=(0, 0)
    radius_v=(0, 0)

    epsilon=0.00001

    if((target_x, target_y)==point):    #if the target point is on the opposite side of the circle
        center=((x+target_x)/2, (y+target_y)/2) #midpoint of diameter
        radius_v=(dist_v[0]/2, dist_v[1]/2) #half of diameter
    elif(abs(unit(dist_v)[0]-math.cos(theta))<epsilon and abs(unit(dist_v)[1]-math.sin(theta))<epsilon):    #if point is collinear and in front of the vector
        return(float("inf"))
    elif (abs(-unit(dist_v)[0]-math.cos(theta))<epsilon and abs(-unit(dist_v)[1]-math.sin(theta))<epsilon): #if point is collinear and behind the vector
        return (-float("inf"))
    else:
        center=circle((x, y), (target_x, target_y), point)  #find the center of the circle with three points
        radius_v=(center[0]-x, center[1]-y)                 #radius as a vector

    radius=mag(radius_v) #the length of radius_v

    if(center_v[0]*radius_v[0]<=0 and center_v[1]*radius_v[1]<=0):  #if the signs don't match, set radius to a right turn
        radius*=-1

    # print("Center: {}".format(center))
    # print("Radius: {}".format(radius))
    return(radius)

def reflect(v, n):  #returns the vector of v reflected across n
    a=(n[0]*2*dot(v, n), n[1]*2*dot(v, n))
    return(sub(a, v))

def circle(p1, p2, p3): #returns the center of a circle that goes through these three points
    coefficients=[[-2*(p1[0]-p2[0]), -2*(p1[1]-p2[1])], [-2*(p3[0]-p2[0]), -2*(p3[1]-p2[1])]]
    values=[p2[0]**2+p2[1]**2-p1[0]**2-p1[1]**2, p2[0]**2+p2[1]**2-p3[0]**2-p3[1]**2]
    center=tuple(np.linalg.solve(coefficients, values))
    return(center)

def unit(v):
    return (v[0]/mag(v), v[1]/mag(v))

def mag(v): #returns the magnitude of v
    return (v[0]**2+v[1]**2)**0.5

def dot(v1, v2):    #returns the dot product of v1 and v2
    return v1[0]*v2[0]+v1[1]*v2[1]

def sub(v1, v2):    #returns the subtraction of v1 from v2
    return (v1[0]-v2[0], v1[1]-v2[1])

################################################################################
##
## BEGIN ALEX'S CODE (which may be formatted to fit your screen)
##
################################################################################
#
#
#def alex_solution(pose, point):
#    arc = Arc(pose, point)
#    return arc.radius
##    robot_pose = Pose((pose[0], pose[1]), pose[2])
##    lookahead = Pose(point)
##    arc = Arc(robot_pose, lookahead)
##    return arc.radius
#
#
##import math
#from dataclasses import dataclass
#import sys
#
#EPSILON = sys.float_info.epsilon
#
#class Arc:
#    '''
#    Defines the arc that connects a robot pose and the lookahead pose
#    '''
#
#    def __init__(self, robot_pose, lookahead_point):
#        self.center = get_arc_center(robot_pose, lookahead_point)
#        if self.center == None:
#            raise ValueError("Unable to connect robot pose "
#                             + f"(X: {robot_pose.vector[0]}"
#                             + f" Y: {robot_pose.vector[1]} "
#                             + f"Theta: {robot_pose.theta})"
#                             + " and lookahead point"
#                             + f"X: {lookahead_point.vector[0]}"
#                             + f" Y: {lookahead_point.vector[1]})")
#        self.radius = get_arc_radius(robot_pose, self.center)
#        direction = get_vector(robot_pose, self.center)
#        self.radius = math.copysign(self.radius, get_direction(robot_pose,
#                                                               lookahead_point))
#        self.length = get_arc_length(robot_pose, lookahead_point,
#                                     self.center, self.radius)
#
#    def __str__(self):
#        return (f"Arc objcet (Center: {self.center}, Radius: {self.radius}"
#                + f" Length: {self.length})")
#
#
#def get_arc_center(robot_pose, lookahead_point):
#    '''
#    Gets the center of the circle that joins the robot
#    and the lookahead
#    '''
#
#    # We know that any circule that contains two points
#    # has it's center on the perpendicular bisector of the
#    # line that joins the two points. So let's find the
#    # equation for that line
#
#    # First find the midpoint and the slope between the two
#    # points
#    midpoint = ((robot_pose[0] + lookahead_point[0])/2,
#                (robot_pose[1] + lookahead_point[1])/2)
#
#    try:
#        slope = ((lookahead_point[1] - robot_pose[1])
#             /(lookahead_point[0] - robot_pose[0]))
#    except ZeroDivisionError:
#        slope = 0
#        a1 = -slope
#        b1 = 1
#        c1 = midpoint[1] - slope * midpoint[0]
#    else:
#
#        # Now we can find the equation for the perpendicular
#        # bisector in the form ax + by = c
#        try:
#            # The slope of the bisector is normal to slope of
#            # the line between the points
#            slope = -(1/slope)
#        except ZeroDivisionError:
#            # If we have a zero dision error the line is vertical
#            a1 = 1
#            b1 = 0
#            c1 = midpoint[0]
#        else:
#            a1 = -slope
#            b1 = 1
#            c1 = midpoint[1] - slope * midpoint[0]
#
#    # Now to find the circle that has a point tanget to the
#    # robot's headinging we find the intersection between
#    # the perpendicular bisector and the line that repesents
#    # the heading normal to the robots current heading
#    try:
#        robot_slope_normal = -1/math.tan(robot_pose[2])
#    except ZeroDivisionError:
#        a2 = 1
#        b2 = 0
#        c2 = robot_pose[0]
#    else:
#        a2 = -robot_slope_normal
#        b2 = 1
#        c2 = robot_pose[1] - robot_slope_normal * robot_pose[0]
#
#
#    # print(f"Bisector: {a1}x + {b1}y = {c1}")
#    # print(f"Normal Robot: {a2}x + {b2}y = {c2}")
#    # print(f"Matrix [[{a1},{b1}],[{a2},{b2}]]")
#
#    # Now we solve the system of equations to find the
#    # intersection and that is the center of our circle
#    determinate = a1 * b2 - a2 * b1
#    # print(f"Determinate: {determinate}")
#    if is_close(determinate, 0):
#        a_close = is_close(a1, a2)
#        b_close = is_close(b1, b2)
#        c_close = is_close(c1, c2)
#        if a_close and b_close and c_close:
#            return midpoint
#        return (float('inf'), float('inf'))
#    return ((c1*b2 - c2*b1)/determinate,
#            (a1*c2 - a2*c1)/determinate)
#
#
#def get_arc_radius(robot_pose, center):
#    '''
#    Get's the radius of the circule that contains the arc
#    connecting the lookahead point and the robot_pose
#    '''
#    d_x = center[0] - robot_pose[0]
#    d_y = center[1] - robot_pose[1]
#    return math.hypot(d_x, d_y)
#
#def get_arc_length(robot_pose, lookahead_point, center, radius):
#    '''
#    Get the length of the arc connecting the robot_pose
#    and the lookahead_point with some pregeneated circle
#    '''
#
#    # As the radius get's of the circle get's bigger
#    # and bigger the length of the arc just converges to
#    # the distance on the line between the robot pose
#    # and the lookahead point so we want to make sure
#    # we are wasting our time calucalting a radius that
#    # is functionaly equivilent to the distance betwene
#    # the points
#    if radius < 1e6:
#
#        # The vector that goes from the center of the circle
#        # to the lookahead point
#        center_to_lookahead = get_vector(center, lookahead_point)
#
#        # The vector that goes from the center of the circle to
#        # to the robot
#        center_to_robot = get_vector(center, robot_pose)
#
#        # If the point is behind the robot we want the oppist of the
#        # angle between the center to lookahead and the center to robot
#        # To determin this we can check the sign of the cross-porbuct
#        # between the normal vertor to the robot and vector that points
#        # from the robot to the lookahead
#
#        # Compute the vector normal to the robot angle
#        normal_angle = bind_radians(robot_pose[2] - math.pi/2)
#        normal_vector = (math.cos(normal_angle), math.sin(normal_angle))
#        robot_to_look = get_vector(robot_pose, lookahead_point)
#
#        # Get the cross product
#        cross_product = (normal_vector[0] * robot_to_look[1]
#                         - normal_vector[1] * robot_to_look[0])
#
#        # get the angle between the vector center_to_robot
#        # and center_to_lookahead
#        dot_product = (center_to_robot[0]
#                       * center_to_lookahead[0] +
#                       center_to_robot[1] * center_to_lookahead[1])
#        ctl_norm = math.hypot(center_to_lookahead[0],
#                              center_to_lookahead[1])
#        ctr_norm = math.hypot(center_to_robot[0],
#                              center_to_robot[1])
#
#        try:
#            cos_angle = dot_product / (ctr_norm * ctl_norm)
#        except ZeroDivisionError:
#            return math.pi * radius
#        if cos_angle == float('nan'):
#            angle = 0.0
#        else:
#            angle = math.acos(limit(cos_angle, -1.0, 1.0))
#        bind_radians(angle)
#
#        if cross_product > 0:
#            return radius * (2.0 * math.pi - abs(angle))
#        else:
#            return radius * abs(angle)
#    else:
#        robot_to_look = get_vector(robot_pose, lookahead_point)
#        return math.hypot(robot_to_look[0], robot_to_look[1])
#
#
### GENERAL UTILITIES
#def interpolate(x, pose1, pose2):
#    '''
#    Intepolates between 2 pose in the direction
#    Pose1 -> Pose2 by x (e.g. an x of 0.5 interpolate
#    halfway between the Poses
#    '''
#    if (x <= 0):
#        return pose1
#    elif x >= 1:
#        return pose1
#    return (x * (pose2[0] - pose1[0]) + pose1[0],
#            x * (pose2[1] - pose1[1]) + pose1[1])
#
#def bind_radians(radians):
#    '''
#    Binds radians between pi and -pi
#    '''
#    pi2 = 2.0 * math.pi
#    radians = radians % pi2
#    radians = (radians + pi2) % pi2
#    if radians > math.pi:
#        radians -= pi2
#    return radians
#
#def is_close(a, b):
#    '''
#    Returns if a two numbers are withen the floating
#    point error bound of eachother
#    '''
#    return (a - EPSILON <= b) and (a + EPSILON >= b)
#
#def signum(num):
#    '''
#    Python doesn't have this function built in so
#    here is an implementation
#    '''
#    return (x > 0) - (x < 0)
#
#
#def limit(num, minimum, maximum):
#    '''
#    Limit some number to be between some
#    minimum and maximum
#    '''
#    return min(maximum, max(minimum, num))
#
#def get_vector(start, end):
#    return ([end[0] - start[0],
#                end[1] - start[1]])
#
#def get_direction(pose, point):
#    pose_to_point = get_vector(pose, point)
#    robot = (math.cos(pose[2]), math.sin(pose[2]))
#    cross = robot[0] * pose_to_point[1] - robot[1] * pose_to_point[0]
#    if cross < 0:
#        return -1
#    else:
#        return 1
#
################################################################################
##
## BAKEOFF!!!
##
################################################################################
#
#def test_known_circles():
#    def try_alex(pose, goal, expected_radius):
#      alex = alex_solution(pose, goal)
#      print(f'Alex: {alex}')
#      utils.assertNear(alex, expected_radius)
#
#    def try_carson(pose, goal, expected_radius):
#      carson = carson_solution(pose, goal)
#      print(f'Carson: {carson}')
#      utils.assertNear(carson, expected_radius)
#
#    def try_it(pose, goal, expected_radius):
#        try_alex(pose, goal, expected_radius)
#        try_carson(pose, goal, expected_radius)
#
#    POSE = (0.0, 1.0, 0.0)
#    GOAL = (0.0, 0.0)
#    EXPECTED_RADIUS = -0.5
#    try_it(POSE, GOAL, EXPECTED_RADIUS)
#
#    POSE = (0.0, -1.0, 0.0)
#    GOAL = (0.0, 0.0)
#    EXPECTED_RADIUS = +0.5
#    try_it(POSE, GOAL, EXPECTED_RADIUS)
#
#    POSE = (0.0, 0.0, math.radians(-90.0))
#    GOAL = (10.0, 0.0)
#    EXPECTED_RADIUS = +5.0
#    try_it(POSE, GOAL, EXPECTED_RADIUS)
#
#    POSE = (377, -117, 4.71)
#    GOAL = (126, 294)
#    EXPECTED_RADIUS = -460.197115043
#    try_it(POSE, GOAL, EXPECTED_RADIUS)
#
#    POSE = (0.0, 0.0, 0.0)  # robot at goal
#    GOAL = (0.0, 0.0)
#    EXPECTED_RADIUS = 0.0
#    try_it(POSE, GOAL, EXPECTED_RADIUS)
#
#    POSE = (1.0, 1.0, math.radians(-135.0))  # directly in front of robot
#    GOAL = (-1.0, -1.0)
#    EXPECTED_RADIUS = float('inf')
#    carson = carson_solution(POSE, GOAL)
#    assert carson == EXPECTED_RADIUS
#
#    POSE = (1.0, 1.0, math.radians(45.0))  # directly behind robot
#    GOAL = (-1.0, -1.0)
#    EXPECTED_RADIUS = float('-inf')
#    carson = carson_solution(POSE, GOAL)
#    assert carson == EXPECTED_RADIUS
#
#    assert False  # reminder to Mr Odom to get Alex to fix his infinities
#
#    ITERATIONS = 10000
#
#    for i in range(ITERATIONS):
#        MIN = -999999
#        MAX = 999999
#        D = 1000
#
#        POSE = (
#            random.randint(MIN, MAX)/D,
#            random.randint(MIN, MAX)/D,
#            random.randint(MIN, MAX)/D)  # radians (may be outside of 2*pi)
#        GOAL = (
#            random.randint(MIN, MAX)/D,
#            random.randint(MIN, MAX)/D)
#
#        carson = carson_solution(POSE, GOAL)
#        alex = alex_solution(POSE, GOAL)
#
#        EPSILON = 0.001  # determined experimentally, still sometimes fails
#        if not utils.isNear(carson, alex, EPSILON):
#            print(f'Carson and Alex disagree')
#            print(f'  POSE: {POSE}')
#            print(f'  GOAL: {GOAL}')
#            print(f'  Carson: {carson}')
#            print(f'  Alex: {alex}')
#            assert False
