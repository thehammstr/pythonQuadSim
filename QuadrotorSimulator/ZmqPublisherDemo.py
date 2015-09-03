#! /usr/bin/python

# Import ZMQ, our fancy-pants networking thingy
import zmq

# Import our custom "state_pb2" protobuf-python code
from protobuf import state_pb2

# Create the ZMQ Context once for any program
context = zmq.Context()

# Create our Publishing socket here
socket = context.socket(zmq.PUB)

# Bind our publishing socket to localhost on some port
socket.bind("tcp://*:5000")

while True:
    pose = state_pb2.PosePb()
    pose.name = 'SimulatedPose'
    pose.valid = True
    pose.timestamp.seconds = 1; # need real timestamp eventually
    pose.translation.x = 1
    pose.translation.y = 2
    pose.translation.z = 3
    pose.rotation.x = 0
    pose.rotation.y = 0
    pose.rotation.z = 0
    pose.rotation.w = 1
    print pose
    socket.send( pose.SerializeToString() )
