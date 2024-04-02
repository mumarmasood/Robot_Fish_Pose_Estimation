# symbolic representation of the robot fish model

# Path: RobotFish_Model_Sym_v1.py

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp
import sympy as sp

# Define the symbolic variables

alpha1, alpha2, delta, xd, yd, l0, l1, l2, d0, d = sp.symbols('self.alpha1 self.alpha2 self.delta xd yd self.l0 self.l1 self.l2 self.d0 self.d')

xd_dot, yd_dot, xd_ddot, yd_ddot = sp.symbols('xd_dot yd_dot xd_ddot yd_ddot')
A_1,A_2,omega, t = sp.symbols('A_1 A_2 omega t')


alpha1 = A_1*sp.sin(omega*t)
alpha2 = A_2*sp.sin(omega*t + sp.pi/2)


xd = l1*sp.cos(alpha1) + l2*sp.cos(alpha2)
yd = l1*sp.sin(alpha1) + l2*sp.sin(alpha2)

xd_dot = sp.diff(xd, t)
yd_dot = sp.diff(yd, t)

xd_ddot = sp.diff(xd_dot, t)
yd_ddot = sp.diff(yd_dot, t)

# print the expression

print('xd = ', xd)
print('yd = ', yd)
print('xd_dot = ', xd_dot)
print('yd_dot = ', yd_dot)
print('xd_ddot = ', xd_ddot)
print('yd_ddot = ', yd_ddot)
