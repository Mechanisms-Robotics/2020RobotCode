import sensors
import networking
from proto import jetson_message_pb2 as jetson_msg

def t256_network_test():
    t256 = sensors.RSPipeline(True, False)
    pub = networking.ZMQServer()
    t256.start()
    try:
        while True:
            t256.wait_for_next_frame()
            update = jetson_msg.JetsonUpdate()
            t256.get_slam_update(update)
            pub.publish_update(update)
    finally:
        pub.close()
        networking.destroy_context()
        t256.stop()

if __name__ == '__main__':
    t256_network_test()