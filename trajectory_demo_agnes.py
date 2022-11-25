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
        '''
        - joint_state: Valores articulares deseados
        '''
        pb.setJointMotorControlArray(self._robot,
                                 range(pb.getNumJoints(self._robot)),
                                 pb.POSITION_CONTROL,
                                 targetPositions = self.joint_state,
                                 forces = [100, 100, 100, 100, 100, 100])
        self.joint_state = joint_state
        
    def move(self, target_joint_state, duration):
        '''
        - target_joint_state: Valores articulares que queremos alcanzar
        - duration: Duración del movimiento
        '''
        _, _, path = self.plan_j5(duration, 
                                 self._dt, 
                                 self.joint_state, 
                                 target_joint_state,
                                 len(self.joint_state)*[0],
                                 len(self.joint_state)*[0],
                                 len(self.joint_state)*[0],
                                 len(self.joint_state)*[0])
        
        input('Press Enter to continue...')
        for i in range(path.shape[0]):
            self.set_joints(path[i])
            time.sleep(self._dt)
            pb.stepSimulation()
            
if __name__ == '__main__':
    # Make an instance of a physic client
    py_client = pb.connect(pb.GUI)

    # Simulation parameters
    pb.setGravity(0, 0, -9.81)
    pb.setRealTimeSimulation(0)

    # Objects
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    ground = pb.loadURDF('plane.urdf')
    #table = pb.loadURDF('table/table.urdf') 
    cube = pb.loadURDF('cube.urdf', basePosition = [1.5, 0.0, 0.125], globalScaling = 0.25)
    agnes_urdf = pb.loadURDF('/URDF/agnes.urdf.xml', useFixedBase=1)
    
    # Instanciating an Agnes robot
    agnes = Agnes(agnes_urdf, 0.01, 0.485, 1.0, 0.74, 0.257564970)
    
     # Movement sequence
    states = (((1.375, 0.2, 0.25),(-np.pi, -np.pi/4, -np.pi/4), 4),
                ((1.625, 0.2, 0.25),(-np.pi, -np.pi/4, -np.pi/4), 4),
                ((1.625, 0.2, 0.35),(-np.pi, -np.pi/4, -np.pi/4), 1),
                ((1.625, -0.2, 0.35),(-np.pi, -np.pi/4, np.pi/4), 2),
                ((1.675, -0.2, 0.25), (-np.pi, -np.pi/4, np.pi/4), 1),
                ((1.375, -0.2, 0.25), (-np.pi, -np.pi/4, np.pi/4), 4)) 
    
    rep = 1
    while rep == 1:
        for i, state in enumerate(states):
            q = agnes.solve(state[0], state[1],-1)
            agnes.move(q, state[2])
        
        rep = int(input('¿Repeat? (1)YES, (ANY OTHER NUMBER)NO: '))
    
    # End program
    input('Press ENTER to stop...')
    pb.disconnect()