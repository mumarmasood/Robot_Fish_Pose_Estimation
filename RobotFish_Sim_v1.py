# Simulates a 2D fish with a non-linear dynamics model solving the differential equations of motion numerically
# author: Umar Masood
# date: 2024-04-01
# version: 1.0
# Bio-inspired Robotics & Control Lab (BRCL) @ UH


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp




class Fish:
    def __init__(self, x, y, psi, delta, alpha1, alpha2):
        self.x = x
        self.y = y
        self.psi = psi
        self.delta = delta
        self.alpha1 = alpha1
        self.alpha2 = alpha2

    def get_state(self):
        return self.x, self.y, self.psi, self.delta, self.alpha1, self.alpha2

    def set_state(self, x, y, psi, delta, alpha1, alpha2):
        self.x = x
        self.y = y
        self.psi = psi
        self.delta = delta
        self.alpha1 = alpha1
        self.alpha2 = alpha2

    def set_shape(self, l0=0.042, l1=0.058, l2=0.022, d0=0.04, d=0.08):
        self.l0 = l0
        self.l1 = l1
        self.l2 = l2
        self.d0 = d0
        self.d = d

    def plot(self, ax, scale = 1):
        # Plotting the pool
        pool = plt.Rectangle((0, 0), _pool_length, _pool_width, color='blue', alpha=0.2)
        ax.add_patch(pool)

        ax.clear()
        ax.set_xlim(0, _pool_length)
        ax.set_ylim(0, _pool_width)
        ax.set_title('2D Robotic Fish Simulator - BRCL @ UH')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.add_patch(pool)

        # define scaled fish parameters
        d0_scaled = self.d0*scale
        d_scaled = self.d*scale
        l0_scaled = self.l0*scale
        l1_scaled = self.l1*scale
        l2_scaled = self.l2*scale

        fish_body = Ellipse((self.x, self.y), d0_scaled, d_scaled, angle=self.psi*180/np.pi, color='red')

        x0_scaled = self.x - 0.5*d0_scaled*np.cos(self.psi)
        y0_scaled = self.y - 0.5*d0_scaled*np.sin(self.psi)
        x1_scaled = x0_scaled - l0_scaled*np.cos(self.psi + self.delta)
        y1_scaled = y0_scaled - l0_scaled*np.sin(self.psi + self.delta)
        x2_scaled = x1_scaled - l1_scaled*np.cos(self.psi + self.delta + self.alpha1)
        y2_scaled = y1_scaled - l1_scaled*np.sin(self.psi + self.delta + self.alpha1)
        x3_scaled = x2_scaled - l2_scaled*np.cos(self.psi + self.delta + self.alpha2)
        y3_scaled = y2_scaled - l2_scaled*np.sin(self.psi + self.delta + self.alpha2)

        fish_tail0 = plt.Line2D([x0_scaled, x1_scaled], [y0_scaled, y1_scaled], color='red')
        fish_tail1 = plt.Line2D([x1_scaled, x2_scaled], [y1_scaled, y2_scaled], color='red')
        fish_tail2 = plt.Line2D([x2_scaled, x3_scaled], [y2_scaled, y3_scaled], color='red')

        ax.add_patch(fish_body)
        ax.add_line(fish_tail0)
        ax.add_line(fish_tail1)
        ax.add_line(fish_tail2)

    def move(self, omega, _del, t):
        self.delta = _del
        A_1 = 0.1
        A_2 = 0.1
    
        self.alpha1 = A_1*np.sin(omega*t)
        self.alpha2 = A_2*np.sin(omega*t + np.pi/2)

        # quarter-chord point of the caudel fin in DSC frame origined at x1, y1 and oriented at angle psi + delta
        xd = self.l1*np.cos(self.alpha1) + self.l2*np.cos(self.alpha2)
        yd = self.l1*np.sin(self.alpha1) + self.l2*np.sin(self.alpha2)

        V_c = 0.1

        xd_dot = -self.l1*np.sin(self.alpha1)*A_1*np.cos(omega*t)*omega - self.l2*np.sin(self.alpha2)*A_2*np.cos(omega*t + np.pi/2)*omega
        yd_dot = self.l1*np.cos(self.alpha1)*A_1*np.cos(omega*t)*omega + self.l2*np.cos(self.alpha2)*A_2*np.cos(omega*t + np.pi/2)*omega
        xd_ddot = -self.l1 * A_1**2 * omega**2 * np.cos(A_1 * np.sin(omega * t)) * np.cos(omega * t)**2 + self.l1 * A_1 * omega**2 * np.sin(A_1 * np.sin(omega * t)) * np.sin(omega * t) - self.l2 * A_2**2 * omega**2 * np.sin(omega * t)**2 * np.cos(A_2 * np.cos(omega * t)) + self.l2 * A_2 * omega**2 * np.sin(A_2 * np.cos(omega * t)) * np.cos(omega * t)
        yd_ddot = -self.l1 * A_1**2 * omega**2 * np.sin(A_1 * np.sin(omega * t)) * np.cos(omega * t)**2 - self.l1 * A_1 * omega**2 * np.cos(A_1 * np.sin(omega * t)) * np.sin(omega * t) - self.l2 * A_2**2 * omega**2 * np.cos(omega * t)**2 * np.cos(A_2 * np.cos(omega * t)) - self.l2 * A_2 * omega**2 * np.cos(A_2 * np.cos(omega * t)) * np.sin(omega * t)



        V_n = - xd_dot*np.sin(self.alpha2) + yd_dot*np.cos(self.alpha2) + V_c*np.sin(self.alpha2)
        V_m = xd_dot*np.cos(self.alpha2) + yd_dot*np.sin(self.alpha2) - V_c*np.cos(self.alpha2)

        V_vect = np.array([V_n, V_m])
        m_hat = np.array([np.cos(self.alpha2), np.sin(self.alpha2)])
        n_hat = np.array([-np.sin(self.alpha2), np.cos(self.alpha2)])

        m_i = 0.5 * np.pi * 1000 * 0.01 * 0.01



        



        # self.plot(ax)



# initiaze constants
_pool_width = 5 # meters
_pool_length = 10 # meters
_pool_depth = 2 # meters
_water_filled_frac = 0.9 # fraction of pool filled with water
_water_filled_depth = _pool_depth * _water_filled_frac # meters


# initialize fish parameters
_fish_l0 = 0.042 # meters
_fish_l1 = 0.058 # meters
_fish_l2 = 0.044 # meters
_fish_L = 0.04 # meters
_fish_d0 = 0.04 # meters
_fish_d = 0.08 # meters
_fish_m = 0.09 # kg
_fish_I = 0.0047 # kg*m^2

# initialize fish state variables
_fish_x = 6 # meters
_fish_y = 2 # meters
_fish_psi = 0 # radians anlge of fish wrt x-axis
_fish_delta = -0.1 # radians angle of fish tail (l0) wrt fish body
_fish_alpha1 = 0.1 # radians angle between l0 and l1
_fish_alpha2 = 0.2 # radians angle between l0 and l2







# Setting up the figure and axis for the plot
fig, ax = plt.subplots()
ax.set_xlim(0, _pool_length)
ax.set_ylim(0, _pool_width)
ax.set_title('2D Robotic Fish Simulator - BRCL @ UH')
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')




# Creating the fish object
roboticfish = Fish(_fish_x, _fish_y, _fish_psi, _fish_delta, _fish_alpha1, _fish_alpha2)
roboticfish.set_shape(_fish_l0, _fish_l1, _fish_l2, _fish_d0*1.5, 0.015)



# Time step for the simulation
dt = 0.1

# Initialization function for the animation
def init():
    roboticfish.plot(ax)

    return []

# Update function for the animation
def update(frame):
    # move to the right
    roboticfish.move(1, 10*np.pi/180, frame*dt)
    
    # update the plot
    
    roboticfish.plot(ax, 28)
    return []

    


# Creating the animation

anim = FuncAnimation(fig, update, init_func=init, frames=200, interval=50, blit=True)

# To display the animation in a Jupyter notebook, use the following line:
# from IPython.display import HTML
# HTML(anim.to_jshtml())

# To save the animation as an mp4 video file, uncomment the line below:
# anim.save('fish_simulation.mp4', writer='ffmpeg')

plt.show()


