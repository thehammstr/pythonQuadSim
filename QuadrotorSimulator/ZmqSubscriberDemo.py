#! /usr/bin/python

# Import ZMQ, our fancy-pants networking thingy
import zmq

# Import our custom "state_pb2" protobuf-python code
from protobuf import state_pb2

# Create the ZMQ Context once for any program
context = zmq.Context()

# Create our Subscribing socket
socket = context.socket(zmq.SUB)

# Connect our Subscribing Socket to Publisher
socket.connect("tcp://127.0.0.1:5000")

# Configure our Subscribing Socket to accept everything
socket.setsockopt(zmq.SUBSCRIBE, "")

while True:
    pose = state_pb2.PosePb()
    pose.ParseFromString(socket.recv());
    print pose