import pybullet as pb
from drivers.solver.solver_agnes import Ik_solver_agnes
import pybullet_data
import numpy as np

# Make an instance of a physic client
phy_client = pb.connect(pb.GUI)

# Simulation parameters
pb.setGravity(0, 0, -9.81)
pb.setRealTimeSimulation(0)

# Objects
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
ground = pb.loadURDF('plane.urdf')
#table = pb.loadURDF('table/table.urdf') 
cube = pb.loadURDF('cube.urdf', basePosition = [1.75, 0.0, 0.25], globalScaling = 0.5)
elbow = pb.loadURDF('URDF/agnes.urdf.xml', useFixedBase=1)

print('\n')
# Inverse kinematics Position 1
print('Pose 1'.center(100, '-'))
input('Press any key to continue...')
o = (1.5, 0.25, 0.5)
rpy = (-np.pi, -np.pi/4, -np.pi/4)
my_solver = Ik_solver_agnes(0.485, 1.0, 0.74, 0.254374022)
print('\n')
q = my_solver.solve(o, rpy, -1)
print('Cinemática inversa'.center(50, '-'))
print('Valores articulares (q): ', q, '\n')
# Execute simulation Position 1
pb.setJointMotorControlArray(elbow, range(6), pb.POSITION_CONTROL, targetPositions = q)
for _ in range(4000):
    pb.stepSimulation()
q_aux, rpy_aux = my_solver._forward_kinematics(q)
print('Cinemática directa'.center(50, '-'))
print(f'Pose con el vector q: O[{q_aux[0]} {q_aux[1]} {q_aux[2]}], RPY[{rpy_aux[0]} {rpy_aux[1]} {rpy_aux[2]}]')

print('\n')
# Inverse kinematics Position 2
print('Pose 2'.center(100, '-'))
input('Press any key to continue...')
print('\n')
o = (1.5, -0.25, 0.5)
rpy = (-np.pi, -np.pi/4, np.pi/4)
q = my_solver.solve(o, rpy, -1)
print('Cinemática inversa'.center(50, '-'))
print('\n')
print('Valores articulares (q): ', q, '\n')
# Execute simulation Position 2
pb.setJointMotorControlArray(elbow, range(6), pb.POSITION_CONTROL, targetPositions = q)
for _ in range(4000):
    pb.stepSimulation()
q_aux, rpy_aux = my_solver._forward_kinematics(q)
print('Cinemática directa'.center(50, '-'))
print(f'Pose con el vector q: O[{q_aux[0]} {q_aux[1]} {q_aux[2]}], RPY[{rpy_aux[0]} {rpy_aux[1]} {rpy_aux[2]}]')

# Disconnect the physic client
print('\n')
input('Press any key to stop...')
print('\n')
pb.disconnect()