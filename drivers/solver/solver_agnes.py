import numpy as np
from numpy import sin, cos 
import transforms3d as t3d

class Ik_solver_agnes:
    def __init__(self, d1, a2, d4, d6):
        self._d1 = d1
        self._a2 = a2
        self._d4 = d4
        self._d6 = d6
        
    def solve(self, o, rpy, conf=1):
        '''
        Método que resuelve la cinemática inversa
        - o: Posición en valores cartesianos (x, y, z)
        - rpy: Rotación del tipo RPY (r, p, y)
        Regresa todos los valores articulares q1, q2, q3, q4, q5, q6 en forma de np.array
        '''
        rt = self._rpy2r(rpy)
        xc, yc, zc = self._wrist_center(o, rt)
        q1, q2, q3 = self._body_ik(xc, yc, zc, conf)
        q4, q5, q6 = self._wrist_ik(q1, q2, q3, rt)

        #o, rpy = self._forward_kinematics([q1, q2, q3, q4, q5, q6])
        #print('o: ', o)
        #print('rpy: ', rpy)
        
        return np.array([q1, q2, q3, q4, q5, q6])

    def _wrist_center(self, o, rt):
        '''
        Método que calcula la ubicación del centro de la muñeca
        - o: Posición del centro de la muñeca en el espacio cartesiano (x, y, z)
        - rt: Rotación en rotación (x, y, z)
        Regresa la ubicación cartesiana del centro de la muñeca (xc, yx, zc)
        '''
        xc = o[0] - self._d6*rt[0][2]
        yc = o[1] - self._d6*rt[1][2]
        zc = o[2] - self._d6*rt[2][2]
        return xc, yc, zc

    def _body_ik(self, xc, yc, zc, conf):
        '''
        Método que calcula la posición del punto O (body)
        - xc: Centro de la muñeca en x
        - yc: Centro de la muñeca en y
        - zc: Centro de la muñeca en z
        - conf: configuración para encontrar el positivo o el negativo de la raíz
        Regresa los valores articulares q1, q2 y q3
        '''
        D = -(xc**2 + yc**2 + (zc - self._d1)**2 - self._a2**2 - self._d4**2)/(2*self._a2*self._d4)

        q3 = np.arctan2(D, conf*np.sqrt(1 - D**2))
        q2 = np.arctan2(zc - self._d1, np.sqrt(xc**2 + yc**2)) - np.arctan2(self._d4*cos(q3), self._a2 - self._d4*sin(q3))
        q1 = np.arctan2(yc, xc)
        return q1, q2, q3

    def _wrist_ik(self, q1, q2, q3, rt):
        '''
        Método que calcula la cinemática inversa de la muñeca (para la orientación del efector)
        - q1: Valor articular del q1
        - q2: Valor articular del q2
        - q3: Valor articular del q3
        - rt: Vector de rotaciones en (x, y, z)
        '''
        rb = np.array([[cos(q1)*cos(q2+q3) ,-sin(q1) , -cos(q1)*sin(q2+q3)],
                       [sin(q1)*cos(q2+q3) , cos(q1) , -sin(q1)*sin(q2+q3)],
                       [sin(q2+q3)         ,   0     ,  cos(q2+q3)        ]])           

        rw = np.dot(np.transpose(rb), rt)

        q4 = np.arctan2(rw[1][2], rw[0][2])
        q5 = np.arctan2(np.sqrt(1 - (rw[2][2])**2), rw[2][2])
        q6 = np.arctan2(rw[2][1], -rw[2][0])

        return q4, -q5, q6 

    def _forward_kinematics(self, q):
        '''
        Método que recibe una tupla **q** 
        Regresa la pose del efector final
        '''
        q1, q2, q3, q4, q5, q6 = q
        Hb = np.array([[cos(q1)*cos(q2+q3) ,-sin(q1) ,-cos(q1)*sin(q2+q3) , self._a2*cos(q1)*cos(q2) ],
                       [sin(q1)*cos(q2+q3) , cos(q1) ,-sin(q1)*sin(q2+q3) , self._a2*sin(q1)*cos(q2) ],
                       [sin(q2+q3)         , 0       , cos(q2+q3)         , self._a2*sin(q2)+self._d1],
                       [0                  , 0       , 0                  , 1                        ]])

        Hw = np.array([
            [cos(q4)*cos(-q5)*cos(q6)-sin(q4)*sin(q6), -cos(q4)*cos(-q5)*sin(q6)-sin(q4)*cos(q6), cos(q4)*sin(q6), cos(q4)*sin(-q5)*self._d6 ],
            [sin(q4)*cos(-q5)*cos(q6)+cos(q4)*sin(q6), -sin(q4)*cos(-q5)*sin(q6)+cos(q4)*cos(q6), sin(q4)*sin(-q5), sin(q4)*sin(-q5)*self._d6 ],
            [-sin(-q5)*cos(q6)                       ,  sin(-q5)*sin(q6)                        , cos(-q5)        , self._d4+cos(-q5)*self._d6],
            [ 0                                     ,  0                                      , 0              , 1                        ]])

        H = np.dot(Hb, Hw)
        o = H[:, 3]
        #(Hb[0, 2], Hb[1, 2], Hb[2, 2])
        rpy = t3d.euler.mat2euler(H[0:3, 0:3], 'sxyz')
        return o, rpy

    def _rpy2r(self, rpy):
        '''
        Método para conversión de rotación rpy a rotación normal uwu
        '''
        y, p, r = rpy

        R = np.array([
            [cos(r)*cos(p), cos(r)*sin(p)*sin(y)-sin(r)*cos(y), cos(r)*sin(p)*cos(y)+sin(r)*sin(y)], 
            [sin(r)*cos(p), sin(r)*sin(p)*sin(y)+cos(r)*cos(y), sin(r)*sin(p)*cos(y)-cos(r)*sin(y)],
            [-sin(p)      ,  cos(p)*sin(y)                    , cos(p)*cos(y)                     ]])

        return R