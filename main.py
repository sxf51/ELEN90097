import time

import mujoco
import mujoco.viewer as viewer
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from casadi import SX, vertcat

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


paused = False

def key_callback(keycode):
    if chr(keycode) == ' ':
        global paused
        paused = not paused

with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
    while viewer.is_running():
        if not paused:
            if (d.qpos[2] < 1):
                d.actuator('motor1').ctrl[0] = 0.52
                d.actuator('motor2').ctrl[0] = 0.52
                d.actuator('motor3').ctrl[0] = 0.52
                d.actuator('motor4').ctrl[0] = 0.52
            else:
                d.actuator('motor1').ctrl[0] = 0.51
                d.actuator('motor2').ctrl[0] = 0.51
                d.actuator('motor3').ctrl[0] = 0.51
                d.actuator('motor4').ctrl[0] = 0.51
            mujoco.mj_step(m, d)
            viewer.sync()


log_count = 0
def control_callback(m, d):
    global log_count, gravity, mass
    _pos = d.qpos
    _vel = d.qvel
    _sensor_data = d.sensordata
    gyro_x = _sensor_data[0]
    gyro_y = _sensor_data[1]
    gyro_z = _sensor_data[2]
    acc_x = _sensor_data[3]
    acc_y = _sensor_data[4]
    acc_z = _sensor_data[5]
    quat_w = _sensor_data[6]
    quat_x = _sensor_data[7]
    quat_y = _sensor_data[8]
    quat_z = _sensor_data[9]
    quat = np.array([quat_x, quat_y, quat_z, quat_w])  # x y z w
    omega = np.array([gyro_x, gyro_y, gyro_z])         # 角速度
    # 构建当前状态
    current_state = np.array([_pos[0], _pos[1], _pos[2], quat[3], quat[0], quat[1], quat[2], _vel[0], _vel[1], _vel[2], omega[0], omega[1], omega[2]])
    # 位置控制模式 目标位点
    goal_position = np.array([0.0, 0.0, 0.5])

    # NMPC Update
    d.actuator('motor1').ctrl[0] = 0.5
    d.actuator('motor2').ctrl[0] = 0.5
    d.actuator('motor3').ctrl[0] = 0.5
    d.actuator('motor4').ctrl[0] = 0.5

    log_count += 1
    if log_count >= 50:
        log_count = 0
        # 这里输出log

def export_model():
    model_name = 'crazyflie'
    # parameters
    g0  = 9.8066     # [m.s^2] accerelation of gravity
    mass  = 33e-3      # [kg] total mass (with one marker)
    Ixx = 1.395e-5   # [kg.m^2] Inertia moment around x-axis
    Iyy = 1.395e-5   # [kg.m^2] Inertia moment around y-axis
    Izz = 2.173e-5   # [kg.m^2] Inertia moment around z-axis
    Cd  = 7.9379e-06 # [N/krpm^2] Drag coef
    Ct  = 3.25e-4    # [N/krpm^2] Thrust coef
    dq  = 65e-3      # [m] distance between motors' center
    l   = dq/2       # [m] distance between motors' center and the axis of rotation

    # 世界坐标系位置  Position of the world coordinate system
    px = SX.sym('px')
    py = SX.sym('py')
    pz = SX.sym('pz')
    # 四元数  quaternion
    q0 = SX.sym('q0')
    q1 = SX.sym('q1')
    q2 = SX.sym('q2')
    q3 = SX.sym('q3')
    # 世界坐标系速度  World coordinate system velocity
    vx = SX.sym('vx')
    vy = SX.sym('vy')
    vz = SX.sym('vz')
    # 机体坐标系角速度  Angular velocity of the machine coordinate system
    wx = SX.sym('wx')
    wy = SX.sym('wy')
    wz = SX.sym('wz')
    # 构建状态向量
    x = vertcat(px, py, pz, q0, q1, q2, q3, vx, vy, vz, wx, wy, wz)

    # 系统控制输入: 四个电机的转速
    w1 = SX.sym('w1')
    w2 = SX.sym('w2')
    w3 = SX.sym('w3')
    w4 = SX.sym('w4')
    u = vertcat(w1, w2, w3, w4)

    # for f_impl
    px_dot = SX.sym('px_dot')
    py_dot = SX.sym('py_dot')
    pz_dot = SX.sym('pz_dot')
    q0_dot = SX.sym('q0_dot')
    q1_dot = SX.sym('q1_dot')
    q2_dot = SX.sym('q2_dot')
    q3_dot = SX.sym('q3_dot')
    vx_dot = SX.sym('vx_dot')
    vy_dot = SX.sym('vy_dot')
    vz_dot = SX.sym('vz_dot')
    wx_dot = SX.sym('wx_dot')
    wy_dot = SX.sym('wy_dot')
    wz_dot = SX.sym('wz_dot')
    # 构建导数状态向量
    xdot = vertcat(px_dot, py_dot, pz_dot, q0_dot, q1_dot, q2_dot, q3_dot, vx_dot, vy_dot, vz_dot, wx_dot, wy_dot, wz_dot)

    # 位置求导
    px_d = vx
    py_d = vy
    pz_d = vz

    # 速度求导
    _thrust_acc_b = Ct*(w1**2 + w2**2 + w3**2 + w4**2) / mass  # 机体坐标系中推力引起的加速度
    # 将机体坐标系推力加速度转换为世界坐标系推力加速度
    # Rwb * [0, 0, _thrust_acc_b]
    _thrust_accx_w = 2*(q1*q3+q0*q2)*_thrust_acc_b
    _thrust_accy_w = 2*(-q0*q1+q2*q3)*_thrust_acc_b
    _thrust_accz_w = 2*(0.5-q1**2-q2**2)*_thrust_acc_b
    vx_d = _thrust_accx_w
    vy_d = _thrust_accy_w
    vz_d = _thrust_accz_w - g0  # 重力加速度

    # 四元数求导
    q0_d = -(q1*wx)/2 - (q2*wy)/2 - (q3*wz)/2
    q1_d =  (q0*wx)/2 - (q3*wy)/2 + (q2*wz)/2
    q2_d =  (q3*wx)/2 + (q0*wy)/2 - (q1*wz)/2
    q3_d =  (q1*wy)/2 - (q2*wx)/2 + (q0*wz)/2
    
    # 机体角速度求导
    # 计算三轴扭矩输入
    mx = Ct*l*(  w1**2 - w2**2 - w3**2 + w4**2)
    my = Ct*l*( -w1**2 - w2**2 + w3**2 + w4**2)
    mz = Cd*  ( -w1**2 + w2**2 - w3**2 + w4**2)
    # 计算角速度导数
    wx_d = (mx + Iyy*wy*wz - Izz*wy*wz)/Ixx
    wy_d = (my - Ixx*wx*wz + Izz*wx*wz)/Iyy
    wz_d = (mz + Ixx*wx*wy - Iyy*wx*wy)/Izz

    # Explicit and Implicit functions
    # 构建显式表达式和隐式表达式
    f_expl = vertcat(px_d, py_d, pz_d, q0_d, q1_d, q2_d, q3_d, vx_d, vy_d, vz_d, wx_d, wy_d, wz_d)
    f_impl = xdot - f_expl

    # algebraic variables
    z = []
    # parameters
    p = []
    
    return
