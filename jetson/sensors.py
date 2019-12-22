import pyrealsense2 as rs
import math
import numpy as np
import utils
import logging

# Constants used for the depth cam
# Note that the STRAM_FPS can only be 30, 60, or 90 depending on the
# resolution we select for each stream.
STREAM_FPS = 60  # FPS

# Note that this resolution is the ideal resolution for the
# Depth camera that we use and it is not suggested to change
# it. If you want a lower resolution to increase the
# speed of post-processing it is suggested that you down
# sample it instead of changing this
DEPTH_RES = (848, 480)  # pixels
BGR_RES = (640, 480)  # pixels

# Controls weather debug statments get printed out
DEBUG_MODE = True
WINDOW_NAME = 'Pipeline'
TUNING_MODE = True
LEFT_ID = 0
RIGHT_ID = 1
USE_WHEEL_ODOM = True

#
SETINGS_DIR = 'settings/'
ODOM_SETTINGS = 'calibration_odometry.json'
kernal = np.ones((5, 5), np.uint8)

last_velocity_hack_todo = 0.0


class RSPipeline:
    '''
    This class handles all RealSense devices
    connected to the robot
    '''

    def __init__(self, use_tracking, depth_camera):
        self.started_ = False
        self.use_tracking = use_tracking
        self.depth_camera = depth_camera
        self.align = None
        self.depth_scale = 0.0
        self.pipeline = rs.pipeline()
        self.frames_ = None
        if USE_WHEEL_ODOM:
            self.wheel_data = None
            logging.info('Wheel odometry is ON.')
        else:
            logging.info('Wheel odometry is OFF.')

    def start(self):
        '''
        Starts the realsense pipeline. Note that when the pipeline
        starts this means that it has restarted all processes.
        (e.g. the tracking camera restarts at pose (0, 0, 0))
        '''
        self.stop()

        self.started_ = True
        cfg = self.generate_config()
        profile = self.pipeline.start(cfg)

        # configure the pipeline

        self.pipeline.stop()  # do I really have to stop and restart it?

        tracking_camera = profile.get_device().first_pose_sensor()

        # No mapping means Visual Inertial Odometry only.  Turning mapping off
        # means that you can't set or even query about pose jumping or
        # relocalization.

        # Experiment: does this clear the map?
        tracking_camera.set_option(rs.option.enable_mapping, 0)
        self.pipeline.start(cfg)
        self.pipeline.stop()

        tracking_camera.set_option(rs.option.enable_mapping, 1)
        tracking_camera.set_option(rs.option.enable_pose_jumping, 0)
        tracking_camera.set_option(rs.option.enable_relocalization, 1)

        self.pipeline.start(cfg)

        i = rs.camera_info(rs.camera_info.firmware_version)
        logging.info('Firmware Version: %s' % tracking_camera.get_info(i))

        o = tracking_camera.get_supported_options()
        logging.info('Supported Options: %s' % o)

        v = tracking_camera.get_option(rs.option.enable_pose_jumping)
        logging.info('Pose jumping option: %s' % v)

        v = tracking_camera.get_option(rs.option.enable_mapping)
        logging.info('Mapping option: %s' % v)

        v = tracking_camera.get_option(rs.option.enable_relocalization)
        logging.info('Relocalization option: %s' % v)

        # start the pipeline

        if self.depth_camera:
            sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = sensor.get_depth_scale()
            self.align = rs.align(rs.stream.color)
        if self.use_tracking and USE_WHEEL_ODOM:
            fp = profile.get_device().as_tm2().first_pose_sensor()
            self.wheel_data = fp.as_wheel_odometer()
            chars = []
            with open(ODOM_SETTINGS) as f:
                for line in f:
                    for c in line:
                        chars.append(ord(c))
            logging.debug('Wheel odometry config:\n\n%s' % chars)
            b = self.wheel_data.load_wheel_odometery_config(chars)
            logging.info('load_wheel_odometery_config returned %s' % b)

    def wait_for_next_frame(self):
        '''
        Wait for the next frame to come in from the realsense pipeline
        '''
        self.frames_ = self.pipeline.wait_for_frames()

    def stop(self):
        '''
        Stops the RealSense pipeline.
        '''
        if self.started_:
            self.pipeline.stop()
            self.started_ = False

    def generate_config(self):
        cfg = rs.config()

        # If the user indicates that they want to use the tracking camera
        # Set the pipeline con fig to get the pose stream
        if self.use_tracking:
            cfg.enable_stream(rs.stream.pose)

        # If the user indicates that they want to use the depth camera
        # then we start the color stream at a 640 x 480 resolution and 60 FPS
        # and the depth stream at 640 x 360 resolution at 60 FPS
        # note that we use bgr8 format for the color stream as this is what
        # plays nicely with opencv
        if self.depth_camera:
            cfg.enable_stream(rs.stream.depth, DEPTH_RES[0], DEPTH_RES[1],
                              rs.format.z16, STREAM_FPS)
            cfg.enable_stream(rs.stream.color, BGR_RES[0], BGR_RES[1],
                              rs.format.bgr8, STREAM_FPS)
        return cfg

    def send_wheel_data(self, left_vel, right_vel):
        if USE_WHEEL_ODOM:
            left_v = rs.vector()
            right_v = rs.vector()
            left_v.z = -left_vel
            right_v.z = -right_vel
            utils.print_occasional('Odometry  Left v: %s  Right v: %s'
                                   % (left_v, right_v))
            self.wheel_data.send_wheel_odometry(LEFT_ID, 0, left_v)
            self.wheel_data.send_wheel_odometry(RIGHT_ID, 0, right_v)

    def get_slam_update(self, update):
        '''
        Grab the slam portion of the current frame and transform it to
        our coordinate system
        '''

        # Only attempt to grab the frame if the pipeline is started, the
        # pipeline is configured for tracking and we have a frame
        if self.started_ and self.use_tracking and self.frames_:
            pose_frame = self.frames_.get_pose_frame()
            if pose_frame:
                pose = pose_frame.get_pose_data()

                # Transforms the realsense coordinates to the robot
                # coordinate system (left is positive y
                # and forward is positive x)
                update.slam_update.x = -pose.translation.z
                update.slam_update.y = -pose.translation.x
                if DEBUG_MODE:
                    utils.print_occasional('Pose x: %s m'
                                           % update.slam_update.x)
                    utils.print_occasional('Pose y: %s m'
                                           % update.slam_update.y)
                rotation = pose.rotation
                # if DEBUG_MODE:
                #     utils.print_occasional(
                #         'Rotation  x: %s  y: %s  z: %s  w: %s' % (
                #             rotation.x, rotation.y, rotation.z, rotation.w))
                update.slam_update.theta = get_robot_rotation(
                    rotation.x, rotation.y, rotation.z, rotation.w)
                if DEBUG_MODE:
                    utils.print_occasional(
                        'Pose theta: %s deg' % utils.radians_to_degrees(
                            update.slam_update.theta))
                update.slam_update.update_epoc = pose_frame.get_timestamp()
                # TODO:  This is not a permanent velocity solution.  Need
                # to create a common codebase for common proto file.
                global last_velocity_hack_todo
                last_velocity_hack_todo = math.sqrt(
                    pose.velocity.x**2 + pose.velocity.z**2)
                if DEBUG_MODE:
                    utils.print_occasional(
                        'Velocity: %s  m/s' % last_velocity_hack_todo)
                    # utils.print_occasional(
                    #    'Mapper confidence (0 - failed  3 - high): %s'
                    #    % pose.mapper_confidence)
                    utils.print_occasional(
                        'Tracker confidence (0 - failed  3 - high): %s'
                        % pose.tracker_confidence)

    def get_rgbd(self):
        if self.started_ and self.depth_camera and self.frames_:
            aligned_frames = self.align.process(self.frames_)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            return (depth_frame, color_frame)

    def get_depth(self):
        if self.started_ and self.depth_camera and self.frames_:
            return np.asanyarray(self.frames_.get_depth_frame().get_data())


def get_robot_rotation(x, y, z, w):
    '''
    Thanks Alex for this function.  Takes quaternion parts and returns the
    rotation of our robot.  POSITIVE y means the robot is facing LEFTWARD.
    '''
    r_x = -(2 * y ** 2) - (2 * z ** 2) + 1
    r_z = (2 * x * z) - (2 * w * y)
    return -math.atan2(r_z, r_x)
