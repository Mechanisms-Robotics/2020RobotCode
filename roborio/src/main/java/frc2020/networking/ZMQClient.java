package frc2020.networking;

import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.List;
import java.util.LinkedList;
import org.zeromq.ZContext;
import org.zeromq.SocketType;
import org.zeromq.ZMQ;
import frc2020.networking.JetsonMessage.JetsonUpdate;
import com.google.protobuf.InvalidProtocolBufferException;

/**
 * Handles reasceing updates from the jetson
 */
public class ZMQClient {
	private final static byte[] TOPIC = "jetsonupdate".getBytes(StandardCharsets.UTF_8);
	private final static byte[] TOPIC_DATA_SEPARATOR = "protobuf".getBytes(StandardCharsets.UTF_8);

	private ZContext context_;
	private ZMQ.Socket socket_;
	private String connect_to_;

	public ZMQClient(ZContext context, String connect_to) {
		context_ = ZContext.shadow(context);
		connect_to_ = "tcp://" + connect_to;
		System.out.println(connect_to_);
		socket_ = context_.createSocket(SocketType.SUB);
		socket_.subscribe(TOPIC);
		socket_.setConflate(true);
	}

	public boolean connect() {
		return socket_.connect(connect_to_);
	}

	public JetsonUpdate getUpdate(boolean useNoBlock) {
		JetsonUpdate update = null;
		byte[] msg = socket_.recv(useNoBlock ? ZMQ.NOBLOCK : 0);
		if (msg == null) return null;
		List<byte[]> data = split(TOPIC_DATA_SEPARATOR, msg);
		try {
			update = JetsonUpdate.parseFrom(data.get(1));
		} catch (InvalidProtocolBufferException e) {
			System.out.println("Malformed Message");
			return null;
		}
		return update;
	}

	private void restart() {
		context_.destroySocket(socket_);
        socket_ = context_.createSocket(SocketType.SUB);
        socket_.connect(connect_to_);
	}


	public static boolean isMatch(byte[] pattern, byte[] input, int pos) {
    	for(int i=0; i< pattern.length; i++) {
        	if(pattern[i] != input[pos+i]) {
            	return false;
        	}
    	}	
    	return true;
	}

	public static List<byte[]> split(byte[] pattern, byte[] input) {
    	List<byte[]> l = new LinkedList<byte[]>();
    	int blockStart = 0;
    	for(int i=0; i<input.length; i++) {
       		if(isMatch(pattern,input,i)) {
          		l.add(Arrays.copyOfRange(input, blockStart, i));
          		blockStart = i+pattern.length;
          		i = blockStart;
       		}
    	}
    	l.add(Arrays.copyOfRange(input, blockStart, input.length ));
    	return l;
	}
}