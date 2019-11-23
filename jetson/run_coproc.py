from proto import jetson_message_pb2 as jetson_msg
import networking
import sensors
import zmq
import path_follower
import utils
import path_generator
import motion_profiler

def main():
    tracking = sensors.RSPipeline(True, False)
    publisher = networking.ZMQServer()
    tracking.start()


    ##################################################################
    # BEGIN MR ODOM EXPERIMENT - VIRTUAL DRIVER
    ##################################################################

    PATH_POINTS = (
        (0, 0),
        (3, -2),
        (7, -2),
        (7, 2),
        (3, 2),
    )

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
            update.drive_signal.demand_left = drive_velocities[0]
            update.drive_signal.demand_right = drive_velocities[1]
            update.drive_signal.demand_type = jetson_msg.DriveSignal.DemandType.Velocity

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
