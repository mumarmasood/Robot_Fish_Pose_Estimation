
import numpy as np
from vpython import canvas, color, vector, cylinder, ellipsoid, rate, box
from scipy.integrate import solve_ivp


class Fish:
    def __init__(self, x, y, psi, delta, alpha1, alpha2, u = 0.0, v = 0, r = -0.0):
        self.x = x
        self.y = y
        self.psi = psi
        self.delta = delta
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.u = u
        self.v = v
        self.r = r

    def get_state(self):
        return self.x, self.y, self.psi, self.delta, self.alpha1, self.alpha2, self.u, self.v, self.r

    def set_state(self, x, y, psi, delta, alpha1, alpha2, u, v, r):
        self.x = x
        self.y = y
        self.psi = psi
        self.delta = delta
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.u = u
        self.v = v
        self.r = r


    def set_shape(self, l0=0.042, l1=0.058, l2=0.022, d0=0.04, d=0.08, L=0.04, m = 0.9, I = 0.0047):
        self.l0 = l0
        self.l1 = l1
        self.l2 = l2
        self.d0 = d0
        self.d = d
        self.L = L
        self.m = m
        self.I = I

    def plot(self, ax, scale = 1):
        # Plotting the pool

        _pool_length = 10.0
        _pool_width = 5.0
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

        fish_body = Ellipse((self.x, self.y), 2*d0_scaled, 0.1, angle=self.psi*180/np.pi, color='red')

        x0_scaled = self.x - d0_scaled*np.cos(self.psi)
        y0_scaled = self.y - d0_scaled*np.sin(self.psi)
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

    def move(self, omega, _del, t, dt):

        # second order system parameters
        rho_w = 1025
        c_x = 0.49
        c_y = 22.5
        c_r = 0.0039
        V_c = 0.08
        s_y = 0.02
        s_x = 0.0025
        k_11 = 0.83
        k_22 = 0.096
        k_55 = 0.55

        D_v = 0.0196
        D_u = 0.45
        D_r = 0.16

        def ode_system(t, states):
            u, v, r = states
            states_dot = np.array([
                (F_x + (self.m + k_22*self.m) * v * r - D_u * u)/(self.m + k_11 * self.m),
                (F_y - (self.m + k_11*self.m) * u * r - D_v * v)/(self.m + k_22 * self.m),
                (F_theta - (k_22 * self.m - k_11 * self.m) * v * u - D_r * r)/(self.I + k_55*self.I)
            ])
            return states_dot
        
        self.delta = _del
        A_1 = 0.33
        A_2 = 0.45
    
        self.alpha1 = A_1*np.sin(omega*t)                       # omega is rad/sec
        self.alpha2 = A_2*np.sin(omega*t + np.pi/2)             # omega is rad/sec

        alpha1_dot = A_1 * omega * np.cos(omega * t)  # The derivative of alpha1 with respect to time
        alpha2_dot = - A_2 * omega * np.sin(omega * t)  # The derivative of alpha2 with respect to time

        alpha1_ddot = - A_1**2 * omega**2 * np.sin(omega * t)  # The second derivative of alpha1 with respect to time
        alpha2_ddot = - A_2**2 * omega**2 * np.cos(omega * t)  # The second derivative of alpha2 with respect to time

        # quarter-chord point of the caudel fin in DSC frame
        xd = self.l1*np.cos(self.alpha1) + self.l2*np.cos(self.alpha2)              # alpha1 and alpha2 are in radians
        yd = self.l1*np.sin(self.alpha1) + self.l2*np.sin(self.alpha2)              # alpha1 and alpha2 are in radians

        xd_dot = -self.l1*np.sin(self.alpha1)*alpha1_dot - self.l2*np.sin(self.alpha2)*alpha2_dot   # xd_dot = d(xd)/dt in m/s
        yd_dot = self.l1*np.cos(self.alpha1)*alpha1_dot + self.l2*np.cos(self.alpha2)*alpha2_dot    # yd_dot = d(yd)/dt in m/s

        xd_ddot =   - self.l1 * (np.sin(self.alpha1) * alpha1_ddot + alpha1_dot**2 * np.cos(self.alpha1)) \
                    - self.l2 * (np.sin(self.alpha2) * alpha2_ddot + alpha2_dot**2 * np.cos(self.alpha2))
        
        yd_ddot =   self.l1 * (np.cos(self.alpha1) * alpha1_ddot - alpha1_dot**2 * np.sin(self.alpha1)) \
                    + self.l2 * (np.cos(self.alpha2) * alpha2_ddot - alpha2_dot**2 * np.sin(self.alpha2))

        V_n = - xd_dot*np.sin(self.alpha2) + yd_dot*np.cos(self.alpha2) + V_c*np.sin(self.alpha2)
        V_m = xd_dot*np.cos(self.alpha2) + yd_dot*np.sin(self.alpha2) - V_c*np.cos(self.alpha2)
 
        V_n_dot = - xd_ddot * np.sin(self.alpha2) \
          - xd_dot * alpha2_dot * np.cos(self.alpha2) \
          + yd_ddot * np.cos(self.alpha2) \
          - yd_dot * alpha2_dot * np.sin(self.alpha2) \
          + V_c * alpha2_dot * np.cos(self.alpha2)  # V_c is constant

        V_vect = np.array([V_n, V_m])
        m_hat = np.array([np.cos(self.alpha2), np.sin(self.alpha2)])
        n_hat = np.array([-np.sin(self.alpha2), np.cos(self.alpha2)])

        m_i = 0.5 * np.pi * rho_w * self.d**2     
        F_rf_d =  0.5 * m_i * V_n**2 * m_hat - m_i * V_n * V_m * n_hat + m_i * self.L * V_n_dot * n_hat

        del_transform = 0.2 * np.array([[np.cos(self.delta), -np.sin(self.delta)], 
                          [np.sin(self.delta), np.cos(self.delta)]])
        
        F = np.dot(del_transform, F_rf_d) # in Wenyu's paper defined as T_x
        F_x = F[0]
        F_y = F[1]

        F_theta = - (self.d0 + (self.l0 + self.l1 + self.l2) * np.cos(self.delta))*F_y + (self.l0 + self.l1 + self.l2) * np.sin(self.delta)*F_x
        
        # append F_theta in the F array
        F = np.append(F, F_theta)

        states = [self.u, self.v, self.r]
        
        ODE_sol = solve_ivp(ode_system, [t - dt, t], states, t_eval=[t])
        
        states = ODE_sol.y[:,-1]

        self.u = states[0]
        self.v = states[1]
        self.r = states[2]

        x_dot = self.u*np.cos(self.psi) - self.v*np.sin(self.psi)
        y_dot = self.u*np.sin(self.psi) + self.v*np.cos(self.psi)
        psi_dot = self.r

        self.x = self.x + x_dot*dt
        self.y = self.y + y_dot*dt
        self.psi = self.psi + psi_dot*dt