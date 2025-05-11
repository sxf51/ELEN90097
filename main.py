import time

import mujoco
import mujoco.viewer as viewer
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from controller import *
from odeSolver import *

m = mujoco.MjModel.from_xml_path('./crazyfile/scene.xml')
d = mujoco.MjData(m)


gravity = 9.81       # gravity (m/s^2)
mass = 0.033            # mass of the quadcopter (kg)
Ct = 3.25e-4            # Motor thrust coefficient (N/krpm^2)
Cd = 7.9379e-6          # Motor reverse torque coefficient (Nm/krpm^2)

arm_length = 0.065/2.0  # The length of the motor lever arm (m)
max_thrust = 0.1573     # Maximum thrust of a single motor (N) (max = 22krpm)
max_torque = 3.842e-03  # Maximum torque of a single motor (Nm) (max = 22krpm)

# 100Hz 10ms 0.01s
dt = 0.01
#print(m.opt.timestep)
m.opt.timestep = dt
next_time = time.time() + dt
m.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4

#d.qpos[3:7] = [0.995, 0, 0.1, 0] # Initial Angle

x0 = [0., 0., 0.1, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
# Time parameters
t0, t_end = 0, 100  # Time range (seconds)
dt = 0.01  # Time step (seconds)
N = int((t_end - t0) / dt)  # Number of steps
t_pts = np.linspace(t0, t_end, N + 1)

paused = False

def key_callback(keycode):
    if chr(keycode) == ' ':
        global paused
        paused = not paused

w = [17, 17, 17, 17]
d.actuator('motor1').ctrl[0] = calc_motor_input(w[0])
d.actuator('motor2').ctrl[0] = calc_motor_input(w[1])
d.actuator('motor3').ctrl[0] = calc_motor_input(w[2])
d.actuator('motor4').ctrl[0] = calc_motor_input(w[3])

sim_data = list()
sim_data.append(control_callback(m, d))
'''
for i in range(N):
    mujoco.mj_step(m, d, nstep = 1) # nstep means the step for each circle
    sim_data.append(control_callback(m, d))
'''



with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
    while viewer.is_running():
        for i in range(N):
            if not paused:
                #time.sleep(dt*5)
                mujoco.mj_step(m, d, nstep = 1)
                sim_data.append(control_callback(m, d))
                viewer.sync()
        break
        


sol_rk45 = solve_ivp(export_model, [t0, t_end], x0, args=(w,), t_eval= t_pts, method="RK45")
sol_rk23 = solve_ivp(export_model, [t0, t_end], x0, args=(w,), t_eval= t_pts, method="RK23")
sol_dop853 = solve_ivp(export_model, [t0, t_end], x0, args=(w,), t_eval= t_pts, method="DOP853")

fig, ax = plt.subplots(2, 1)
sim_data = np.array(sim_data)

ax[0].plot(t_pts, sol_rk45.y[9, :], "r-", label = "RK45")
ax[0].plot(t_pts, sim_data[:, 9], "b-", label = "simulation")
ax[0].set(xlabel='time/s', ylabel='The velocity in the Z-axis direction')

ax[0].legend()
ax[0].grid()

ax[1].plot(t_pts, sol_rk45.y[2, :], "r-", label = "RK45")
ax[1].plot(t_pts, sim_data[:, 2], "b-", label = "simulation")
ax[1].set(xlabel='time/s', ylabel='Z-axis coordinate')

ax[1].legend()
ax[1].grid()

plt.show()
print(sim_data[-1, 9])
