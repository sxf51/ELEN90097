import time

import mujoco
import mujoco.viewer as viewer
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from casadi import SX, vertcat

from controller import *
from odeSolver import *

m = mujoco.MjModel.from_xml_path('./crazyfile/scene.xml')
d = mujoco.MjData(m)


gravity = 9.8066        # 重力加速度 单位m/s^2
mass = 0.033            # 飞行器质量 单位kg
Ct = 3.25e-4            # 电机推力系数 (N/krpm^2)
Cd = 7.9379e-6          # 电机反扭系数 (Nm/krpm^2)

arm_length = 0.065/2.0  # 电机力臂长度 单位m
max_thrust = 0.1573     # 单个电机最大推力 单位N (电机最大转速22krpm)
max_torque = 3.842e-03  # 单个电机最大扭矩 单位Nm (电机最大转速22krpm)

# 仿真周期 100Hz 10ms 0.01s
dt = 0.01
# 更改仿真的时间步长
#print(m.opt.timestep)
m.opt.timestep = dt
next_time = time.time() + dt

paused = False

def key_callback(keycode):
    if chr(keycode) == ' ':
        global paused
        paused = not paused

w = [0.52, 0.52, 0.52, 0.52]
d.actuator('motor1').ctrl[0] = w[0]
d.actuator('motor2').ctrl[0] = w[1]
d.actuator('motor3').ctrl[0] = w[2]
d.actuator('motor4').ctrl[0] = w[3]

x0 = [0., 0., 0.1, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
# Time parameters
t0, t_end = 0, 0.5  # Time range (seconds)
dt = 0.01  # Time step (seconds)
N = int((t_end - t0) / dt)  # Number of steps
t_pts = np.linspace(t0, t_end, N + 1)

sim_data = list()
sim_data.append(control_callback(m, d)[2])
for i in range(N):
    mujoco.mj_step(m, d, nstep = 1) # nstep means 每次前进几步
    sim_data.append(control_callback(m, d)[2])
'''
with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
    while viewer.is_running():
        for i in range(N):
            if not paused:
                if (d.qpos[2] < 1):
                    d.actuator('motor1').ctrl[0] = 1
                    d.actuator('motor2').ctrl[0] = 1
                    d.actuator('motor3').ctrl[0] = 1
                    d.actuator('motor4').ctrl[0] = 1
                else:
                    d.actuator('motor1').ctrl[0] = 0.51
                    d.actuator('motor2').ctrl[0] = 0.51
                    d.actuator('motor3').ctrl[0] = 0.51
                    d.actuator('motor4').ctrl[0] = 0.51
                sleep_time = next_time - time.time()
                next_time = time.time() + dt
                if sleep_time > 0:
                    time.sleep(sleep_time)
                mujoco.mj_step(m, d, nstep = 1) # nstep means 每次前进几步
                sim_data.append(control_callback(m, d)[2])
                viewer.sync()
        paused = True

'''

solution = solve_ivp(export_model, [t0, t_end], x0, args=(w,), t_eval= t_pts, method='RK45')

plt.plot(t_pts, solution.y[2, :], "s-", label = "RK45")
plt.plot(t_pts, sim_data, "o-", label = "simulation")

plt.legend()
plt.grid()
plt.show()