from proto import jetson_message_pb2 as jetson_msg
import networking
import sensors
import utils
import path_generator
import motion_profiler
import kinematics


ODOMETRY_DEAD_ZONE = 0.0  # m/s


def main():
    tracking = sensors.RSPipeline(True, False)
    publisher = networking.ZMQServer()
    subscriber = networking.ZMQClient()
    subscriber.connect()
    tracking.start()

    ##################################################################
    # BEGIN MR ODOM EXPERIMENT - VIRTUAL DRIVER
    ##################################################################

    PATH_POINTS = [
        (0, 0)
    ]

    PATH_POINTS.extend([
        (8, -1.2),
        (8, 1.4),
        (2, 1.4),
        (2, -1.2),
    ]*100)

    path = path_generator.generate_path_from_points(PATH_POINTS)
    motion_profile = motion_profiler.MotionProfile(path)

    ##################################################################
    # END MR ODOM EXPERIMENT
    ##################################################################

    try:
        while True:
            update = jetson_msg.JetsonUpdate()
            tracking.wait_for_next_frame()
            tracking.get_slam_update(update)
            rio_update = subscriber.rio_update()
            if rio_update:
                if rio_update.odometry_update:
                    wheel_left = rio_update.odometry_update.left
                    wheel_right = rio_update.odometry_update.right
                    if abs(wheel_left) <= ODOMETRY_DEAD_ZONE:
                        wheel_left = 0.0
                    if abs(wheel_right) <= ODOMETRY_DEAD_ZONE:
                        wheel_right = 0.0
                    tracking.send_wheel_data(wheel_left, wheel_right)

            ##################################################################
            # BEGIN MR ODOM EXPERIMENT - VIRTUAL DRIVER
            ##################################################################

            # HEADING HOLD

            # (dl, dr) = path_follower.heading_hold(
            #     utils.degrees_to_radians(-30.0),
            #     update.slam_update.theta)

            # LINE HOLD

            # (dl, dr) = path_follower.line_hold(
            #     update.slam_update.x,
            #     update.slam_update.y,
            #     update.slam_update.theta,
            #     (0, 0),
            #     (1, 0))

            # MR ODOM'S PATH FOLLOWER

            # PATH = (
            #     (0, 0),
            #     (1, 0),
            #     (2, 0),
            #     (3, 0),
            #     (4, 0),
            #     (5, 0),
            #     (5, 1.5),
            #     (4, 1.5),
            #     (3, 1.5),
            #     (2, 1.5),
            #     (1, 1.5),
            #     (0, 1.5),
            #     (0, 0)
            # )
            #
            # (dl, dr) = path_follower.path_follow(
            #     update.slam_update.x,
            #     update.slam_update.y,
            #     update.slam_update.theta,
            #     PATH)

            # 4910 PURE PURSUIT

            pose = (
                update.slam_update.x,
                update.slam_update.y,
                update.slam_update.theta)
            velocity = sensors.last_velocity_hack_todo
            drive_velocities = path.follow_path(pose, velocity, motion_profile)

            # EXPERIMENT FOR TESTING KINEMATICS

            # VELOCITY = 1.0  # m / s
            # TURN_RADIUS = 2.0  # m
            # drive_velocities = kinematics.find_drive_velocities(
            #     VELOCITY, TURN_RADIUS)

            # send drive velocities to Rio

            update.drive_signal.demand_left = drive_velocities[0]
            update.drive_signal.demand_right = drive_velocities[1]
            update.drive_signal.demand_type = (
                jetson_msg.DriveSignal.DemandType.Velocity)

            utils.print_occasional(
                'Vl: %s  Vr %s' % (drive_velocities))

            utils.call_increment()

            ##################################################################
            # END MR ODOM EXPERIMENT
            ##################################################################

            publisher.publish_update(update)
            # time.sleep(0.001)  # TODO: limit publication rate
    finally:
        publisher.close()
        tracking.stop()


if __name__ == "__main__":
    main()
