import numpy as np
import matplotlib.pyplot as plt

# Parameters
L = 4.0  # wheelbase
theta_in = np.deg2rad(20.0)  # steering angle, in radians
v = 10.0  # velocity, m/s
dt = 0.05  # time step, s
sim_time = 10.0  # total simulation time, s

# Initialization
steps = int(sim_time / dt)
x = np.zeros(steps)
y = np.zeros(steps)
psi = np.zeros(steps)

# Simulation loop
for i in range(1, steps):
    x[i] = x[i-1] + v * np.cos(psi[i-1]) * dt
    y[i] = y[i-1] + v * np.sin(psi[i-1]) * dt
    psi[i] = psi[i-1] + v / L * np.tan(theta_in) * dt

# Plotting the trajectory
    
plt1 = plt.figure(1)
plt.plot(x, y, 'r-', label='Trajectory')
plt.plot(x[0], y[0], 'go', label='Start')
plt.plot(x[-1], y[-1], 'bx', label='End')
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.title('Vehicle Trajectory using Bicycle Model')
plt.legend()
plt.axis('equal')
plt.grid(True)
# plt.show()


from scipy.integrate import solve_ivp

# Define the model as a system of ODEs
def bicycle_model(t, state, v, L, theta):
    x, y, psi = state
    dxdt = v * np.cos(psi)
    dydt = v * np.sin(psi)
    dpsidt = v / L * np.tan(theta)
    return [dxdt, dydt, dpsidt]

# Initial conditions and parameters
initial_state = [0, 0, 0]  # Starting at the origin with an orientation of 0 radians
time_span = [0, 10]  # Solve from t=0 to t=10 seconds
# Solve the ODE
solution = solve_ivp(bicycle_model, time_span, initial_state, args=(v, L, theta_in), dense_output=True)

# solution.y will contain the solution [x(t), y(t), psi(t)] at the time points requested
t = np.linspace(0, 10, 100)
z = solution.sol(t)

# Plot the trajectory
plt.figure(2)
plt.plot(z[0], z[1], 'r-', label='Trajectory')
plt.plot(z[0, 0], z[1, 0], 'go', label='Start')
plt.plot(z[0, -1], z[1, -1], 'bx', label='End')
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.title('Vehicle Trajectory using Dynamic Bicycle Model')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()


