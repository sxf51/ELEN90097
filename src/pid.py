import numpy as np

def pid_controller(y, p_des, v_des=[0, 0, 0], yaw_des=0):
    px, py, pz = y[0:3]
    q0, q1, q2, q3 = y[3:7]
    vx, vy, vz = y[7:10]
    wx, wy, wz = y[10:13]

    g = 9.81
    mass = 0.033
    Ct = 3.25e-4
    Cd = 7.9379e-6
    l = 0.065 / 2

    Kp_pos = np.array([2.0, 2.0, 4.0])
    Kd_pos = np.array([1.0, 1.0, 3.0])
    Kp_ang = np.array([200, 200, 120])

    pos_error = np.array(p_des) - np.array([px, py, pz])
    vel_error = np.array(v_des) - np.array([vx, vy, vz])

    acc_cmd = Kp_pos * pos_error + Kd_pos * vel_error + np.array([0, 0, g])
    thrust = mass * acc_cmd[2]

    phi_des = acc_cmd[1] / g   # around x (roll)
    theta_des = -acc_cmd[0] / g  # around y (pitch)

    phi = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
    theta = np.arcsin(2*(q0*q2 - q3*q1))
    psi = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))

    ang_error = np.array([
        phi_des - phi,
        theta_des - theta,
        yaw_des - psi
    ])
    torque_cmd = Kp_ang * ang_error

    u1 = thrust
    u2 = torque_cmd[0]
    u3 = torque_cmd[1]
    u4 = torque_cmd[2]

    A = np.array([
        [Ct, Ct, Ct, Ct],
        [Ct*l, -Ct*l, -Ct*l, Ct*l],
        [-Ct*l, -Ct*l, Ct*l, Ct*l],
        [-Cd, Cd, -Cd, Cd]
    ])
    A_inv = np.linalg.inv(A)
    u = np.array([u1, u2, u3, u4])
    w_squared = np.dot(A_inv, u)

    w = np.sqrt(np.clip(w_squared, 0, None))

    return w.tolist()
