import mujoco
import numpy as np

gravity = 9.81
mass = 0.033
Ct = 3.25e-4
Cd = 7.9379e-6

arm_length = 0.065/2.0
max_thrust = 0.1573
max_torque = 3.842e-03 

# 100Hz 10ms 0.01s
dt = 0.01

# Calculate the motor thrust based on the motor speed.
def calc_motor_force(krpm):
    global Ct
    return Ct * krpm**2

# Calculate the normalized input of the motor based on its rotational speed.
def calc_motor_input(krpm):
    if krpm > 22:
        krpm = 22
    elif krpm < 0:
        krpm = 0
    _force = calc_motor_force(krpm)
    _input = _force / max_thrust
    if _input > 1:
        _input = 1
    elif _input < 0:
        _input = 0
    return _input

# Load model callback function
def load_callback(m=None, d=None):
    mujoco.set_mjcb_control(None)
    m = mujoco.MjModel.from_xml_path('./crazyfile/scene.xml')
    d = mujoco.MjData(m)
    if m is not None:
        mujoco.set_mjcb_control(lambda m, d: control_callback(m, d))
    return m, d

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
    omega = np.array([gyro_x, gyro_y, gyro_z])

    current_state = np.array([_pos[0], _pos[1], _pos[2], quat[3], quat[0], quat[1], quat[2], _vel[0], _vel[1], _vel[2], omega[0], omega[1], omega[2]])

    goal_position = np.array([0.0, 0.0, 0.5])

    return current_state

'''
    log_count += 1
    if log_count >= 50:
        log_count = 0
        # log
'''

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


