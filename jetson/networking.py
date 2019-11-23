import zmq
from google.protobuf.message import DecodeError
from proto import roborio_message_pb2 as rio_msg
from proto import jetson_message_pb2 as jetson_msg

context_ = zmq.Context()

def destroy_context():
    if context_ != None:
        context_.term()
        context_ = None

JETSON_TOPIC = b'jetsonupdate'
RIO_TOPIC = b'rioupdate'
TOPIC_DATA_SEPARATOR = b'protobuf'
DEFAULT_PORT_PUB = '5556'
DEFAULT_PORT_SUB = '5555'

class ZMQServer:

	def __init__(self, port = DEFAULT_PORT_PUB):
		self.connect_to_ = "tcp://*:" + port
		self.socket_ = context_.socket(zmq.PUB)
		self.socket_.bind(self.connect_to_)

	def publish_update(self, update):
		data = update.SerializeToString()
		self.socket_.send(JETSON_TOPIC + TOPIC_DATA_SEPARATOR + data)

	def close(self):
		self.socket_.close()

class ZMQClient:

    def __init__(self, ip = "10.49.10.2", port = DEFAULT_PORT_SUB):
        self.connect_to_ = "tcp://" + ip + ":" + port
        self.sub_ = context_.socket(zmq.SUB)
        self.sub_.setsockopt(zmq.CONFLATE, 1)
        
        
    def connect(self):
        self.sub_.connect(self.connect_to_)
        print ("Subsciber connected to " + self.connect_to_)
        self.sub_.subscribe(RIO_TOPIC)
    
    def rio_update(self):
        try:
            msg = self.sub_.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            return None
        data = msg.split(TOPIC_DATA_SEPARATOR, 1)[1]
        update = rio_msg.RioUpdate()
        try:
            update.ParseFromString(data) 
        except DecodeError:
            print("Unable to decode rio message")
            return None
        return update
        
    def close(self):
        self.sub_.close()
