'''
Written by yufeng.wu0902@gmail.com

Python class for FK/IK calculations on Q8bot.
'''

import math
import numpy as np
from scipy.optimize import fsolve

# d - distance between motors; l1/l1p - upper linkage length; l2/l2p - lower linkage length
# Unit is in mm.
class k_solver:
    def __init__(self, d = 19.5, l1 = 25, l2 = 40, l1p = 25, l2p = 40):
        self.d = d
        self.l1 = l1
        self.l2 = l2
        self.l1p = l1p
        self.l2p = l2p
        self.prev_ik = [45, 135]
        self.prev_est = [self.d/2, (self.l1 + self.l2)]

    # Check whether a solution exist. To be completed later.
    def ik_check(self, x, y):
        if y < 0:
            return False
        return True

    # Solve inverse kinematics at an given end effector position
    def ik_solve(self, x, y, deg = True, rounding = 3):
        try:
            c1 = math.sqrt((x - self.d)**2 + y**2)
            c2 = math.sqrt(x**2 + y**2)
            a1 = math.acos((c1**2 + self.d**2 - c2**2) / (2*c1*self.d))
            a2 = math.acos((c2**2 + self.d**2 - c1**2) / (2*c2*self.d))
            b1 = math.acos((c1**2 + self.l1**2 - self.l2**2) / (2*c1*self.l1))
            b2 = math.acos((c2**2 + self.l1p**2 - self.l2p**2) / (2*c2*self.l1p))
            q1 = math.pi - a1 - b1
            q2 = a2 + b2
            if deg:
                q1, q2 = q1*180/math.pi, q2*180/math.pi
            self.prev_ik = [q1, q2]
            return np.round(q1, rounding), np.round(q2, rounding), True
        except:
            return self.prev_ik[0], self.prev_ik[1], False
    
    # Check whether a solution exist. To be completed later.
    def fk_check(self):
        return True
    
    # Solve inverse kinematics at an given end effector position
    def fk_solve(self, q1, q2, deg = True, rounding = 3):
        if deg:
            angles = (self._deg2rad(q1), self._deg2rad(q2))
        x, y = fsolve(self._fk_calc, self.prev_est, args=angles)
        self.prev_est = [x, y]
        return round(x, rounding), round(y, rounding)

    #-------------------#
    # Private Functions #
    #-------------------#
    def _fk_calc(self, x, *angle):
        q1, q2 = angle
        Xa = self.l1 * math.cos(q1) + self.d
        Ya = self.l1 * math.sin(q1)
        Xb = self.l1p * math.cos(q2)
        Yb = self.l1p * math.sin(q2)
        return [x[0]*x[0] - 2*x[0]*Xa + Xa**2 + x[1]*x[1] - 2*x[1]*Ya + Ya**2 - self.l2**2,
                x[0]*x[0] - 2*x[0]*Xb + Xb**2 + x[1]*x[1] - 2*x[1]*Yb + Yb**2 - self.l2p**2]
    
    def _rad2deg(self, ang_rad):
        return ang_rad*180/math.pi
    
    def _deg2rad(self, ang_deg):
        return ang_deg*math.pi/180
