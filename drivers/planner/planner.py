import numpy as np
class Planner:
    def __init__(self):
        pass
    
    def plan_j3(self, tf, dt, q0, qf, v0, vf, a0, af):
        q0 = np.array(q0)
        qf = np.array(qf)
        v0 = np.array(v0)
        vf = np.array(vf)
        a0 = np.array(a0)
        af = np.array(af)
        
        #Time vector
        t = np.arange(0,tf+dt,dt)
        
        # Polynomial coefficients
        c0 = q0
        c1 = v0
        c2 = a0/2
        c3 = ((20*qf-20*q0)-(8*vf+12*v0)*tf+(af-3*a0)*tf**2)/(2*tf**3)
        c4 = -((30*qf-30*q0)-(14*vf+16*v0)*tf+(2*af-3*a0)*tf**2)/(2*tf**4)
        c5 = ((12*qf-12*q0)-(6*vf+6*v0)*tf+(af-a0)*tf**2)/(2*tf**5)
        
        # Trajectory
        q = np.dot(np.array([t**0, t, t**2, t**3, t**4, t**5]).transpose(),np.array([c0, c1, c2, c3, c4, c5]))
        v = np.dot(np.array([t**0 , t, t**2, t**3, t**4]).transpose(), np.array([c1, 2*c2, 3*c3, 4*c4, 5*c5]))
        a = np.dot(np.array([t**0, t, t**2, t**3]).transpose(), np.array([2*c2, 6*c3, 12*c4, 20*c5]))

        return a, v, q

    
    def plan_line(self, tf, dt, p0, pf):
        dt = int(tf/dt)
        # Position and orientation
        o0 = np.array(p0[:3])
        of = np.array(pf[:3])
        a0 = np.array(np.deg2rad(p0[3:6]))
        af = np.array(np.deg2rad(pf[3:6]))
        # Auxiliar vector coefficients
        v = of - o0
        # x-vector and time vector
        x = np.linspace(p0[0], pf[0], dt)
        t = (x - p0[0])/ v[0]
        # y and z vectors
        y = p0[1] + v[1]*t
        z = p0[2] + v[2]*t
        # Wrist
        a = np.linspace(p0[3], pf[3], dt)
        b = np.linspace(p0[4], pf[4], dt)
        c = np.linspace(p0[5], pf[5], dt)
                        
        return x, y, z, a, b, c

if __name__ == '__main__':
    # Initial and final values
    q0 = np.array([0])
    qf = np.array([50])
    v0 = np.zeros(len(q0))
    vf = np.zeros(len(q0))
    a0 = np.zeros(len(q0))
    af = np.zeros(len(q0)) 
    tf = 5
    dt = 0.01

    planner = Planner()
    a, v, q = planner.plan_j3(tf, 0.01, q0, qf, v0, vf, a0, af)
    t = np.arange(0,tf+dt,dt)

    # plotting
    import matplotlib.pyplot as plt 
    plt.plot(t,q,label='Position')
    plt.plot(t, v, label='Velocity')
    plt.plot(t, a, label='Acceleration')
    plt.legend()
    plt.grid()
    plt.title('Joint position (5th degree polynomial)')
    plt.show() 