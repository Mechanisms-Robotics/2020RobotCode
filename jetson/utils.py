import math


call_count = 0


def print_occasional(s):  # for debugging
    global call_count
    PERIOD = 400  # wait this many calls
    if call_count % PERIOD == 0:
        print(s)


def call_increment():  # for debugging
    global call_count
    print_occasional('')
    call_count += 1


def distance_between_points(p1, p2):
    a = p1[0] - p2[0]
    b = p1[1] - p2[1]
    return math.sqrt(a*a + b*b)


def assertNear(a, b):
    EPSILON = 0.0001
    assert abs(a - b) < EPSILON


def get_robot_distance_from_line(p1, p2, r):
    '''
    Gets the distance of the robot from a line defined by p1 and p2.  This
    may return a negative distance.  Imagine that we want the robot to travel
    parallel with the line in the direction FROM p1 TO p2.  A negative
    distance means that the robot is LEFT of the line, while a positive
    distance means that the robot is RIGHT of the line.
    '''
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    num = dy*r[0] - dx*r[1] + p2[0]*p1[1] - p2[1]*p1[0]
    denom = math.sqrt(dy*dy + dx*dx)
    return num/denom


def test_get_robot_distance_from_line():
    p1 = (0, 0)
    p2 = (0, 1)
    r = (2, 2)
    d = get_robot_distance_from_line(p1, p2, r)
    assertNear(d, +2.0)

    r = (-2, -2)
    d = get_robot_distance_from_line(p1, p2, r)
    assertNear(d, -2.0)

    p1 = (5, 5)
    p2 = (6, 6)
    r = (6, 5)
    d = get_robot_distance_from_line(p1, p2, r)
    assertNear(d, math.sqrt(2.0)/2)

    d = get_robot_distance_from_line(p2, p1, r)
    assertNear(d, -math.sqrt(2.0)/2)


def get_heading_of_line(p1, p2):
    '''
    Imagine that we have a line that is directional from p1 to p2.  This gets
    the heading of that line.
    '''
    xd = p2[0] - p1[0]
    yd = p2[1] - p1[1]
    return math.atan2(yd, xd)


def test_get_heading_of_line():
    p1 = (10, 0)
    p2 = (100, 0)
    theta = get_heading_of_line(p1, p2)
    assertNear(theta, 0.0)

    theta = get_heading_of_line(p2, p1)
    assertNear(abs(theta), math.pi)

    p1 = (0, 10)
    p2 = (0, 100)
    theta = get_heading_of_line(p1, p2)
    assertNear(theta, math.pi/2)

    theta = get_heading_of_line(p2, p1)
    assertNear(theta, -math.pi/2)

    p1 = (1, 1)
    p2 = (2, 2)
    theta = get_heading_of_line(p1, p2)
    assertNear(theta, math.pi/4)

    theta = get_heading_of_line(p2, p1)
    assertNear(theta, -3*math.pi/4)


def radians_to_degrees(r):
    return r*180.0/math.pi


def degrees_to_radians(d):
    return d*math.pi/180.0
