import pybullet as pb
import pybullet_data
from drivers.solver.solver_agnes import Ik_solver_agnes
from drivers.planner.planner import Planner
import time
import numpy as np

class Agnes(Ik_solver_agnes, Planner):
    def __init__(self, robot, dt, d1, a2, d4, d6):
        super().__init__(d1, a2, d4, d6)

        self._robot = robot
        self._dt = dt
        self.gripper_state = False
        self.joint_state = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        
    def set_joints(self, joint_state):
        pb.setJointMotorControlArray(self._robot,
                                 range(pb.getNumJoints(self._robot)),
                                 pb.POSITION_CONTROL,
                                 targetPositions = self.joint_state,
                                 forces = [100, 100, 100, 100, 100, 100])
        self.joint_state = joint_state

    def move(self, joint_target):
        self.set_joints(joint_target)
        time.sleep(self._dt)
        pb.stepSimulation()
    
if __name__ == '__main__':
    # Make an instance of a physic client
    py_client = pb.connect(pb.GUI)

    # Simulation parameters
    pb.setGravity(0, 0, -9.81)
    pb.setRealTimeSimulation(0)
    dt = 0.01

    # Objects
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    ground = pb.loadURDF('plane.urdf') 
    cube = pb.loadURDF('cube.urdf', basePosition = [1.5, 0.0, 0.125], globalScaling = 0.25)
    agnes_urdf = pb.loadURDF('/URDF/agnes.urdf.xml', useFixedBase=1)

    # Instanciating an Agnes robot
    agnes = Agnes(agnes_urdf, 0.01, 0.485, 1.0, 0.74, 0.257564970)

    # Trajectory
    vx, vy, vz, ap, bp, cp = agnes.plan_line(5, dt, (1.225,0,1.22444, 0, 0, 0), (1.5,0,1.22444, 0, 0, 0))
    
    input('Press ENTER to continue...')
    for x, y, z, a, b, c in zip(vx, vy, vz, ap, bp, cp):
        joint_state = agnes.solve((x, y, z),(a, b, c), -1)
        # print(f'joint_state {type(joint_state)}')
        agnes.move(joint_state)
        
    # Trajectory
    vx, vy, vz, ap, bp, cp = agnes.plan_line(5, dt,(1.5,0,1.22444, 0, 0, 0), (1.25,.25,1.22444, 0, 0, 45))
    
    input('Press ENTER to continue...')
    for x, y, z, a, b, c in zip(vx, vy, vz, ap, bp, cp):
        joint_state = agnes.solve((x, y, z),(a, b, c), -1)
        # print(f'joint_state {type(joint_state)}')
        agnes.move(joint_state)
        
    # End program
    input('Press ENTER to stop..')
    pb.disconnect()
    