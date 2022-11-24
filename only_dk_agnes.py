from drivers.solver.solver_agnes import Ik_solver_agnes

class Only_dk_agnes(Ik_solver_agnes):
    def __init__(self, d1, a2, d4, d6):
        super().__init__(d1, a2, d4, d6)

agnes_dk = Only_dk_agnes(0.485, 1.0, 0.74, 0.254374022)
q = (0,0,0,0,0,0)
print(f'{agnes_dk._forward_kinematics(q)}')       