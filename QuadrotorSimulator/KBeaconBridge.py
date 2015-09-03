#! /usr/bin/python

# Import ZMQ, our fancy-pants networking thingy
import zmq
import time

# Import our custom "kzmq_pb2" protobuf-python code
from protobuf.py import kzmq_pb2

# Create the ZMQ Context once for any program
context = zmq.Context()

# Create our Subscribing socket
socket = context.socket(zmq.REQ)

# Connect our Subscribing Socket to Publisher
socket.connect("tcp://localhost:5100")

def lookup_cached(topic_name):
    # request
    topic_req = kzmq_pb2.TopicPb()
    topic_req.action = kzmq_pb2.TopicPb.LOOKUP
    topic_req.name = topic_name
    socket.send(topic_req.SerializeToString())
    # wait for reply
    result_data = socket.recv()
    topic_res = kzmq_pb2.TopicPb()
    topic_res.ParseFromString(result_data)
    return topic_res.address

def lookup_blocking(topic_name):
    print "[enter] lookup_blocking ", topic_name
    while (True):
        address = lookup_cached(topic_name)
        if (address):
            print "[exit] lookup_blocking ", topic_name
            return address
        time.sleep(1)

# for now we only pass port in address, fix later
def publish(name, type, address):
    topic = kzmq_pb2.TopicPb()
    topic.action = kzmq_pb2.TopicPb.PUBLISH
    topic.name = name
    topic.type = type
    topic.address = address
    socket.send(topic.SerializeToString())
    data = socket.recv() # just recieve nothing because req/rep requires it to be balanced

#publish("test.vicon_pose", "drone.pose", "8274");
#print lookup_blocking("estoril.vicon_pose")
