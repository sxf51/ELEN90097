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
    