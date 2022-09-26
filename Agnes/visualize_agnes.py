import numpy as np
import pybullet as pb

#Making an instance of a physical client
client = pb.connect(pb.GUI)

# Simulation parameters
pb.setGravity(0,0,-9.81)
pb.setRealTimeSimulation(0)

#Charging the objects
robot = pb.loadURDF("URDF/agnes.urdf.xml", useFixedBase=1)

#Ending the program
input('Press any key to stop...')
pb.disconnect()
