import numpy as np
from odeSolver import export_model

def pid_controller(y, p_des, v_des=[0, 0, 0], yaw_des=0):
    # 状态解包
    px, py, pz = y[0:3]
    q0, q1, q2, q3 = y[3:7]
    vx, vy, vz = y[7:10]
    wx, wy, wz = y[10:13]

    # 参数
    g = 9.81
    mass = 0.033
    Ct = 3.25e-4
    Cd = 7.9379e-6
    l = 0.065 / 2  # 电机到中心的距离

    # PID参数（可以调节）
    Kp_pos = np.array([2.0, 2.0, 4.0])
    Kd_pos = np.array([1.0, 1.0, 3.0])
    Kp_ang = np.array([200, 200, 120])

    # 期望误差
    pos_error = np.array(p_des) - np.array([px, py, pz])
    vel_error = np.array(v_des) - np.array([vx, vy, vz])

    # 推力控制（z轴方向）
    acc_cmd = Kp_pos * pos_error + Kd_pos * vel_error + np.array([0, 0, g])
    thrust = mass * acc_cmd[2]

    # 将期望姿态设为与位置控制相关的简单控制律（仅 pitch 和 roll）
    phi_des = acc_cmd[1] / g   # around x (roll)
    theta_des = -acc_cmd[0] / g  # around y (pitch)

    # 当前姿态（四元数转欧拉角）
    # 简化计算：从四元数直接估算phi/theta/psi
    phi = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
    theta = np.arcsin(2*(q0*q2 - q3*q1))
    psi = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))

    # 姿态控制
    ang_error = np.array([
        phi_des - phi,
        theta_des - theta,
        yaw_des - psi
    ])
    torque_cmd = Kp_ang * ang_error

    # 解算四电机转速平方（单位: krpm²）
    u1 = thrust
    u2 = torque_cmd[0]
    u3 = torque_cmd[1]
    u4 = torque_cmd[2]

    # 控制分配矩阵逆解
    # [u1, u2, u3, u4]^T = A * [w1², w2², w3², w4²]^T
    A = np.array([
        [Ct, Ct, Ct, Ct],
        [Ct*l, -Ct*l, -Ct*l, Ct*l],
        [-Ct*l, -Ct*l, Ct*l, Ct*l],
        [-Cd, Cd, -Cd, Cd]
    ])
    A_inv = np.linalg.inv(A)
    u = np.array([u1, u2, u3, u4])
    w_squared = np.dot(A_inv, u)

    # 防止负值、取平方根
    w = np.sqrt(np.clip(w_squared, 0, None))  # 单位: krpm

    return w.tolist()
