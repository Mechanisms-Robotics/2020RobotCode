import math
import utils
import arcs
import path_generator
import kinematics


def heading_hold(desired_heading, current_heading):  # angles are in RADIANS
    '''
    Return wheel demands that will use proportional steering to turn us toward
    desired heading.  Remember that POSITIVE is LEFT.
    '''
    e = (current_heading - desired_heading) % (2*math.pi)
    heading_error = e - 2*math.pi if e > math.pi else e
    P = 1
    correction = P * heading_error / (math.pi / 2)  # positive means turn LEFT
    D = 0.5  # base demand
    dl = D + correction
    dr = D - correction
    return (dl, dr)  # TODO: Restrict demands to [-1, 1]?


def test_heading_hold():
    # on zero line
    (dl, dr) = heading_hold(0.0, 0.0)
    assert dl == dr

    # on anti-zero line
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(+180.0),
      utils.degrees_to_radians(+180.0))
    assert dl == dr
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(-180.0),
      utils.degrees_to_radians(+180.0))
    assert dl == dr
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(+180.0),
      utils.degrees_to_radians(-180.0))
    assert dl == dr
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(-180.0),
      utils.degrees_to_radians(-180.0))
    assert dl == dr

    # heading +- 180, robot +-179
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(+180.0),
      utils.degrees_to_radians(+179.0))
    assert dr > dl  # left turn
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(+180.0),
      utils.degrees_to_radians(-179.0))
    assert dr < dl  # right turn
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(-180.0),
      utils.degrees_to_radians(+179.0))
    assert dr > dl  # left turn
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(-180.0),
      utils.degrees_to_radians(-179.0))
    assert dr < dl  # right turn

    # heading +- 179, robot +-179
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(+179.0),
      utils.degrees_to_radians(+179.0))
    assert dr == dl
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(+179.0),
      utils.degrees_to_radians(-179.0))
    assert dr < dl  # right turn
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(-179.0),
      utils.degrees_to_radians(+179.0))
    assert dr > dl  # left turn
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(-179.0),
      utils.degrees_to_radians(-179.0))
    assert dr == dl

    # check abs(...) > 180
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(-181.0),
      utils.degrees_to_radians(179.0))
    assert dr == dl
    (dl, dr) = heading_hold(
      utils.degrees_to_radians(-179.0),
      utils.degrees_to_radians(181.0))
    assert dr == dl


def line_hold(current_x, current_y, current_heading, p1, p2):
    '''
    current_x and current_y are in METERS, with x FORWARD and POSITIVE
    y LEFT.  This intercepts and holds a directional line defined by
    p1 and p2 in the direction from p1 to p2 (units in meters and radians).
    '''
    P = 1  # radians per meter
    line_heading = utils.get_heading_of_line(p1, p2)  # radians
    distance = utils.get_robot_distance_from_line(
        p1,
        p2,
        (current_x, current_y))  # meters
    heading_correction = P * distance  # positive means turn left into line
    desired_heading = line_heading + heading_correction
    return heading_hold(desired_heading, current_heading)


def test_line_hold():
    current_x = 100
    current_y = 0
    current_heading = 0
    p1 = (0, 0)
    p2 = (1, 0)
    (dl, dr) = line_hold(current_x, current_y, current_heading, p1, p2)
    assert dl == dr

    current_y = 0.1  # turn right
    (dl, dr) = line_hold(current_x, current_y, current_heading, p1, p2)
    utils.assertNear(dl, 0.56366197)  # will change if P changes
    utils.assertNear(dr, 0.43633802)

    current_x = 10
    current_y = 10
    current_heading = math.pi/4
    p1 = (0, 0)
    p2 = (1, 1)
    (dl, dr) = line_hold(current_x, current_y, current_heading, p1, p2)
    assert dl == dr


def path_follow(current_x, current_y, current_heading, path):
    '''
    current_x and current_y are in METERS, with x FORWARD and POSITIVE
    y is LEFT.  This will intercept the tangent line to the closest
    path segment (kind of).
    '''

    # determine the point to which we are closest
    assert(len(path) > 1)
    closest_point_index = 0
    closest_point_distance = utils.distance_between_points(
        path[0], (current_x, current_y))
    for i in range(1, len(path) - 1):  # excludes last point so we have a line
        d = utils.distance_between_points(path[i], (current_x, current_y))
        if d <= closest_point_distance:
            closest_point_index = i
            closest_point_distance = d

    # use the line hold to intercept and hold the line
    return line_hold(
        current_x,
        current_y,
        current_heading,
        path[closest_point_index],
        path[closest_point_index + 1])

###############################################################################
#
# CODE BELOW IS PURE PURSUIT IMPLEMENTATION BY MR. ODOM
#
###############################################################################


LOOKAHEAD_TIME = 0.5  # s (tunable)


class Path:
    # tested implicitly by other tests
    def __init__(self, path_segments):
        '''
        Path segments is a list of paths contained by this path.  For primitive
        paths (such as LinearPathSegment), self.path_segments may be None,
        in which case implement overrides for functions that use it.
        '''

        self.path_segments = path_segments
        self.distance_on_path = 0.0  # meters along path

    # tested by nothing (need a visualizer to test visually)
    def follow_path(self, pose, velocity, motion_profile):
        '''
        This is the main path following function.  Given a pose in the form
        (x, y, theta), the robot's velocity, and the motion profile, this
        returns new drive velocities, (velocity left, velocity right).  This
        function should be called iteratively as the path runs.
        '''

        # print(f'  Velocity: {velocity}')

        # update the position along the path based on current robot state
        # print(f'  Distance on path before: {self.distance_on_path}')
        self.update_distance_on_path(pose, velocity)
        # print(f'  Distance on path after: {self.distance_on_path}')

        # determine the lookahead point
        lookahead_point = self.get_point_on_path(
            self.distance_on_path + LOOKAHEAD_TIME*velocity)
        # print(f'  Lookahead point: {lookahead_point}')

        # return the drive velocities to steer toward the lookahead point
        turn_radius = arcs.current_solution(pose, lookahead_point)
        # print(f'  Turn radius: {turn_radius}')
        desired_velocity = motion_profile.get_desired_velocity(
            self.distance_on_path)
        # print(f'  Desired velocity: {desired_velocity}')
        if turn_radius == float('inf'):
            velocities = (desired_velocity, desired_velocity)
        elif turn_radius == float('-inf'):
            velocities = kinematics.find_drive_velocities(
                desired_velocity, -99999999)  # TODO: think about this behavior
        else:
            velocities = kinematics.find_drive_velocities(
                desired_velocity, turn_radius)
        # print(f'  Velocities: {velocities}')
        return velocities  # (velocity left, velocity right)

    # tested by test_update_distance_on_path
    def update_distance_on_path(self, robot_pose, robot_velocity):
        '''
        Given a robot pose, (x, y, theta), this determines the distance along
        the path that best corresponds with the robot pose.  Since the robot
        pose may be close to more than one point along the path, this uses the
        last known pose of the robot on the path and the robot velocity to
        create a window on the path and finds the best point in that window.
        Updates self.distance_on_path, returning nothing.
        '''

        assert self.path_segments is not None
        assert len(self.path_segments) > 0

        # Determine which path segments are in the window.  A path segment is
        # in the window if its end point is greater than the last distance
        # along the path, and its start point is less than or equal to the
        # window endpoint on the path.  The reason that the inequalities
        # are that way is so that we have a tendency to favor path segments
        # ahead of us.  Note that the window endpoint may actually be beyond
        # the last segment, which is okay.  For convenience, each segment is
        # stored with metadata as a tuple like (segment,
        # cumulative length of all previous segments).

        endpoint_distance = robot_velocity * 2*LOOKAHEAD_TIME  # m
        endpoint = self.distance_on_path + endpoint_distance  # m

        length_of_previous_segments = 0.0  # same as start of each segment
        segments_in_window = []

        for segment in self.path_segments:
            segment_length = segment.get_length()
            segment_start = length_of_previous_segments  # for code readability
            segment_end = segment_start + segment_length

            on_path = (
                (segment_end > self.distance_on_path) and
                (segment_start <= endpoint)
            )

            if on_path:
                segments_in_window.append(
                    (segment, length_of_previous_segments))

            length_of_previous_segments += segment_length

        # At this point we have the segments in the window.  Now we determine
        # the point of closest approach for each segment in the window.  At the
        # same time, we use the point of closest approach to determine the
        # robot's distance from the segment and we calculate the distance along
        # the segment of the point of closest approach.  The output of this
        # step is a list of tuples like this:
        #
        # [ (
        #    (segment, cumulative length of all previous segments),
        #    robot's distance from segment,
        #    distance along segment of point of closest approach
        #   ),
        # ... ]

        assert len(segments_in_window) > 0

        segment_data_list = []

        for segment_and_cumulative_length in segments_in_window:
            segment = segment_and_cumulative_length[0]
            (p, d) = segment.get_closest_point_on_path(robot_pose)
            segment_data_list.append((
                segment_and_cumulative_length,
                utils.distance_between_points(p, robot_pose[:-1]),
                d))

        # Finally, we use the distance along the segments to find the closest
        # segment and determine the distance along the path.  We remember this
        # value for next time and return it.  In the event that two segments
        # are equally close, the strict inequality makes the earliest segment
        # the robot's segment.

        assert len(segment_data_list) > 0

        closest_segment_data = segment_data_list[0]

        for segment_data in segment_data_list[1:]:
            if segment_data[1] < closest_segment_data[1]:
                closest_segment_data = segment_data

        self.distance_on_path = (closest_segment_data[0][1]
                                 + closest_segment_data[2])

    # tested by test_get_length
    def get_length(self):
        '''
        Return the length of this path in meters.

        This should be overriden in any base class.
        '''

        assert self.path_segments is not None
        assert len(self.path_segments) > 0

        length = 0.0
        for path in self.path_segments:
            length += path.get_length()

        return length

    # tested by nothing since this is abstract (for now)
    def get_closest_point_on_path(self, robot_pose):
        '''
        Given the robot pose, (x, y, theta), determine the point on the path
        closest to the pose.  If there are multiple points on the path the
        same distance from the pose, this returns the first such point.  Return
        value is ( (x, y) of point, distance along path of this point ).

        This should be overriden in any base class.
        '''

        raise NotImplementedError(
            'implement in any paths that may be contained by other paths')

    # tested by test_get_point_on_path
    def get_point_on_path(self, distance_on_path):
        '''
        Given a distance along the path, return the corresponding (x, y).
        Extrapolates if the distance is beyond the end of the path.  This
        should be overriden in any child class.
        '''

        assert self.path_segments is not None
        assert len(self.path_segments) > 0

        # search segment by segment until we find the point's segment

        length_of_previous_segments = 0.0

        for segment in self.path_segments:
            segment_length = segment.get_length()
            total_distance = length_of_previous_segments + segment_length
            if total_distance > distance_on_path:
                # we have reached the segment
                return segment.get_point_on_path(
                    distance_on_path - length_of_previous_segments)
            length_of_previous_segments = total_distance

        # If we get to this point, the distance is beyond the end of the path,
        # so require the last segment to extrapolate for us.

        return self.path_segments[-1].get_point_on_path(
            distance_on_path
            - length_of_previous_segments
            + self.path_segments[-1].get_length())


TEST_POINTS = (
    (1, 1),
    (2, 2),
    (2, 3),
    (3, 3),
    (4, 2),
    (4, 1),
    (3, 0))


def test_update_distance_on_path():
    TEST_PATH = path_generator.generate_path_from_points(TEST_POINTS)

    assert TEST_PATH.distance_on_path == 0

    ROBOT_POSE = (1.5, 1.5, 0)
    ROBOT_VELOCITY = 1
    EXPECTED_DISTANCE = math.sqrt(2)/2
    TEST_PATH.update_distance_on_path(ROBOT_POSE, ROBOT_VELOCITY)
    assert TEST_PATH.distance_on_path == EXPECTED_DISTANCE

    ROBOT_POSE = (2, 2, 0)
    ROBOT_VELOCITY = 1
    EXPECTED_DISTANCE = math.sqrt(2)
    TEST_PATH.update_distance_on_path(ROBOT_POSE, ROBOT_VELOCITY)
    assert TEST_PATH.distance_on_path == EXPECTED_DISTANCE

    ROBOT_POSE = (2.1, 2.9, 0)
    ROBOT_VELOCITY = 1
    EXPECTED_DISTANCE = 2.3142135623730953  # experimentally determined
    TEST_PATH.update_distance_on_path(ROBOT_POSE, ROBOT_VELOCITY)
    assert TEST_PATH.distance_on_path == EXPECTED_DISTANCE

    ROBOT_POSE = (2.5, 3.1, 0)
    ROBOT_VELOCITY = 1
    EXPECTED_DISTANCE = 2.914213562373095  # experimentally determined
    TEST_PATH.update_distance_on_path(ROBOT_POSE, ROBOT_VELOCITY)
    assert TEST_PATH.distance_on_path == EXPECTED_DISTANCE

    ROBOT_POSE = (4, 2, 0)
    ROBOT_VELOCITY = 2
    EXPECTED_DISTANCE = 2 + 2*math.sqrt(2)
    TEST_PATH.update_distance_on_path(ROBOT_POSE, ROBOT_VELOCITY)
    assert TEST_PATH.distance_on_path == EXPECTED_DISTANCE

    ROBOT_POSE = (2.8, -0.5, 0)
    ROBOT_VELOCITY = 2
    EXPECTED_DISTANCE = TEST_PATH.get_length()
    TEST_PATH.update_distance_on_path(ROBOT_POSE, ROBOT_VELOCITY)
    assert TEST_PATH.distance_on_path == EXPECTED_DISTANCE


def test_get_length():
    TEST_PATH = path_generator.generate_path_from_points(TEST_POINTS)
    EXPECTED_LENGTH = 3*math.sqrt(2) + 3
    length = TEST_PATH.get_length()
    utils.assertNear(length, EXPECTED_LENGTH)


def test_get_point_on_path():
    TEST_PATH = path_generator.generate_path_from_points(TEST_POINTS)

    DISTANCE = 0
    EXPECTED_POINT = (1, 1)
    point = TEST_PATH.get_point_on_path(DISTANCE)
    assert point == EXPECTED_POINT

    DISTANCE = math.sqrt(2)/2
    EXPECTED_POINT = (1.5, 1.5)
    point = TEST_PATH.get_point_on_path(DISTANCE)
    assert point == EXPECTED_POINT

    DISTANCE = math.sqrt(2)
    EXPECTED_POINT = (2, 2)
    point = TEST_PATH.get_point_on_path(DISTANCE)
    assert point == EXPECTED_POINT

    DISTANCE = 2*math.sqrt(2) + 2.5
    EXPECTED_POINT = (4, 1.5)
    point = TEST_PATH.get_point_on_path(DISTANCE)
    assert point == EXPECTED_POINT

    DISTANCE = TEST_PATH.get_length()
    EXPECTED_POINT = (3, 0)
    point = TEST_PATH.get_point_on_path(DISTANCE)
    assert point == EXPECTED_POINT

    DISTANCE = TEST_PATH.get_length() + math.sqrt(2)
    EXPECTED_POINT = (2, -1)
    point = TEST_PATH.get_point_on_path(DISTANCE)
    utils.assertNear(0, utils.distance_between_points(EXPECTED_POINT, point))


class LinearPathSegment(Path):
    # tested by various implicit tests
    def __init__(self, p1, p2):
        super().__init__(None)
        self.p1 = p1
        self.p2 = p2
        self.length = utils.distance_between_points(self.p1, self.p2)

    # tested by test_get_linear_path_segment_length
    def get_length(self):
        return self.length

    # tested by test_get_closest_point_on_linear_path
    def get_closest_point_on_path(self, robot_pose):
        # calculate the closest point on the line defined by p1 and p2
        p = (robot_pose[0], robot_pose[1])
        closest_point_on_line = utils.get_closest_point_on_line(
            self.p1, self.p2, p)

        # make sure the point is on the segment

        min_x = min(self.p1[0], self.p2[0])
        max_x = max(self.p1[0], self.p2[0])
        min_y = min(self.p1[1], self.p2[1])
        max_y = max(self.p1[1], self.p2[1])

        is_on_segment = (closest_point_on_line[0] >= min_x and
                         closest_point_on_line[0] <= max_x and
                         closest_point_on_line[1] >= min_y and
                         closest_point_on_line[1] <= max_y)

        if is_on_segment:
            return (closest_point_on_line, utils.distance_between_points(
                closest_point_on_line, self.p1))

        # we need to return an endpoint

        distance_from_p1 = utils.distance_between_points(self.p1, p)
        distance_from_p2 = utils.distance_between_points(self.p2, p)

        if distance_from_p1 < distance_from_p2:
            return (self.p1, 0)
        return (self.p2, self.get_length())

    # tested by test_get_point_on_linear_path_segment
    def get_point_on_path(self, distance_on_path):
        # print(f'    Distance on path: {distance_on_path}')
        assert self.p1 != self.p2
        dx = self.p2[0] - self.p1[0]
        dy = self.p2[1] - self.p1[1]
        d = math.sqrt(dx*dx + dy*dy)
        x = self.p1[0] + dx*distance_on_path/d
        y = self.p1[1] + dy*distance_on_path/d
        return (x, y)


def test_get_linear_path_segment_length():
    P1 = (0, 0)
    P2 = (0, 2)
    EXPECTED_LENGTH = 2
    length = LinearPathSegment(P1, P2).get_length()
    assert length == EXPECTED_LENGTH

    P1 = (2, 0)
    P2 = (0, 0)
    EXPECTED_LENGTH = 2
    length = LinearPathSegment(P1, P2).get_length()
    assert length == EXPECTED_LENGTH

    P1 = (20, -20)
    P2 = (10, 10)
    EXPECTED_LENGTH = 10*math.sqrt(10)
    length = LinearPathSegment(P1, P2).get_length()
    utils.assertNear(length, EXPECTED_LENGTH)


def test_get_closest_point_on_linear_path():
    P1 = (0, 0)
    P2 = (1, 0)
    ROBOT_POSE = (0, 0, 0)
    EXPECTED_POINT = (0, 0)
    EXPECTED_DISTANCE = 0
    (point, distance) = LinearPathSegment(P1, P2).get_closest_point_on_path(
        ROBOT_POSE)
    assert point == EXPECTED_POINT
    assert distance == EXPECTED_DISTANCE

    P1 = (0, 0)
    P2 = (0, 1)
    ROBOT_POSE = (0, 1, 0)
    EXPECTED_POINT = (0, 1)
    EXPECTED_DISTANCE = 1
    (point, distance) = LinearPathSegment(P1, P2).get_closest_point_on_path(
        ROBOT_POSE)
    assert point == EXPECTED_POINT
    assert distance == EXPECTED_DISTANCE

    P1 = (10, 10)
    P2 = (20, 20)
    ROBOT_POSE = (15, 15, 0)
    EXPECTED_POINT = (15, 15)
    EXPECTED_DISTANCE = 5*math.sqrt(2)
    (point, distance) = LinearPathSegment(P1, P2).get_closest_point_on_path(
        ROBOT_POSE)
    assert point == EXPECTED_POINT
    assert distance == EXPECTED_DISTANCE

    P1 = (10, 10)
    P2 = (20, 20)
    ROBOT_POSE = (15, 16, 0)
    EXPECTED_POINT = (15.5, 15.5)
    EXPECTED_DISTANCE = 11/math.sqrt(2)  # about 7.78
    (point, distance) = LinearPathSegment(P1, P2).get_closest_point_on_path(
        ROBOT_POSE)
    assert point == EXPECTED_POINT
    assert distance == EXPECTED_DISTANCE

    P1 = (10, 10)
    P2 = (20, 20)
    ROBOT_POSE = (25, 25, 0)
    EXPECTED_POINT = P2
    EXPECTED_DISTANCE = 10*math.sqrt(2)
    (point, distance) = LinearPathSegment(P1, P2).get_closest_point_on_path(
        ROBOT_POSE)
    assert point == EXPECTED_POINT
    assert distance == EXPECTED_DISTANCE

    P1 = (20, 20)
    P2 = (10, 10)
    ROBOT_POSE = (6, 6, 0)
    EXPECTED_POINT = P2
    EXPECTED_DISTANCE = 10*math.sqrt(2)
    (point, distance) = LinearPathSegment(P1, P2).get_closest_point_on_path(
        ROBOT_POSE)
    assert point == EXPECTED_POINT
    assert distance == EXPECTED_DISTANCE


def test_get_point_on_linear_path_segment():
    P1 = (0, 0)
    P2 = (0, 2)
    DISTANCE_ON_PATH = 1
    EXPECTED_POINT = (0, 1)
    point = LinearPathSegment(P1, P2).get_point_on_path(DISTANCE_ON_PATH)
    assert point == EXPECTED_POINT

    P1 = (0, 0)
    P2 = (0, 2)
    DISTANCE_ON_PATH = 3
    EXPECTED_POINT = (0, 3)
    point = LinearPathSegment(P1, P2).get_point_on_path(DISTANCE_ON_PATH)
    assert point == EXPECTED_POINT

    P1 = (0, 0)
    P2 = (0, -2)
    DISTANCE_ON_PATH = 1
    EXPECTED_POINT = (0, -1)
    point = LinearPathSegment(P1, P2).get_point_on_path(DISTANCE_ON_PATH)
    assert point == EXPECTED_POINT

    P1 = (0, 0)
    P2 = (2, 0)
    DISTANCE_ON_PATH = 1
    EXPECTED_POINT = (1, 0)
    point = LinearPathSegment(P1, P2).get_point_on_path(DISTANCE_ON_PATH)
    assert point == EXPECTED_POINT

    P1 = (0, 0)
    P2 = (-2, 0)
    DISTANCE_ON_PATH = 1
    EXPECTED_POINT = (-1, 0)
    point = LinearPathSegment(P1, P2).get_point_on_path(DISTANCE_ON_PATH)
    assert point == EXPECTED_POINT

    P1 = (10, 20)
    P2 = (20, 40)
    DISTANCE_ON_PATH = 5
    EXPECTED_POINT = (10 + math.sqrt(5), 20 + 2*math.sqrt(5))
    point = LinearPathSegment(P1, P2).get_point_on_path(DISTANCE_ON_PATH)
    assert point == EXPECTED_POINT

    P1 = (1, 1)
    P2 = (2, 2)
    DISTANCE_ON_PATH = math.sqrt(2)
    EXPECTED_POINT = (2, 2)
    point = LinearPathSegment(P1, P2).get_point_on_path(DISTANCE_ON_PATH)
    assert point == EXPECTED_POINT

    P1 = (2, 2)
    P2 = (1, 1)
    DISTANCE_ON_PATH = math.sqrt(2)
    EXPECTED_POINT = (1, 1)
    point = LinearPathSegment(P1, P2).get_point_on_path(DISTANCE_ON_PATH)
    assert point == EXPECTED_POINT
