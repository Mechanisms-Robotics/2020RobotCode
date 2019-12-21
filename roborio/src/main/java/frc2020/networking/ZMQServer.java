package frc2020.networking;

import java.nio.charset.StandardCharsets;
import org.zeromq.ZContext;
import org.zeromq.SocketType;
import org.zeromq.ZMQ;
import frc2020.networking.RoborioMessage.RioUpdate;

public class ZMQServer {
    private final static byte[] TOPIC = "rioupdate".getBytes(StandardCharsets.UTF_8);
    private final static byte[] TOPIC_DATA_SEPARATOR = "protobuf".getBytes(StandardCharsets.UTF_8);
    private final static int DEFAULT_PORT = 5556;

    private ZContext context_;
    private ZMQ.Socket socket_;

    public ZMQServer(ZContext context) {
        context_ = ZContext.shadow(context);
        socket_ = context_.createSocket(SocketType.PUB);
    }

    public void connect() {
        socket_.bind("tcp://*:" + DEFAULT_PORT);
    }

    public void publishUpdate(RioUpdate message) {
        byte[] data = makeMsg(TOPIC, message.toByteArray());
        socket_.send(data);
    }

    public void closeServer() {
        context_.destroySocket(socket_);
        context_.destroy();
    }

    private static byte[] makeMsg(byte[] topic, byte[] data) {
        byte[] ret = new byte[topic.length + TOPIC_DATA_SEPARATOR.length + data.length];
        for (int i = 0; i < ret.length; ++i) {
            if (i < topic.length) {
                ret[i] = topic[i];
            } else if (i < (topic.length + TOPIC_DATA_SEPARATOR.length)) {
                ret[i] = TOPIC_DATA_SEPARATOR[i - topic.length];
            } else {
                ret[i] = data[i - (topic.length + TOPIC_DATA_SEPARATOR.length)];
            }
	    }
	    return ret;
    }
}
