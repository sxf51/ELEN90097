import time

import mujoco
import mujoco.viewer as viewer
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from src.controller import *
from src.dynamic import *
from src.validate import *
import sqlite3
import redis


# Database configuration
file_name = '.\data\quadcopter.db'

# SQLite
conn = sqlite3.connect(file_name)
cur = conn.cursor()


def table_exists(db_conn: sqlite3.Connection, table_name: str) -> bool:
    """
    Check if a table exists in SQLite database
    
    :param db_conn: Database connection object
    :param table_name: Table name to check
    :return: Whether the table exists
    """
    cursor = db_conn.execute(
        "SELECT name FROM sqlite_master "
        "WHERE type='table' AND name=?",
        (table_name,)
    )
    return cursor.fetchone() is not None

def truncate_all_tables(db_path):
    with sqlite3.connect(db_path) as conn:
        cursor = conn.cursor()
        
        # get table names
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
        tables = [row[0] for row in cursor.fetchall()]
        
        conn.execute("BEGIN IMMEDIATE TRANSACTION")
        try:
            # Disable foreign key constraints
            conn.execute("PRAGMA foreign_keys=OFF")
            
            # delate tables
            for table in tables:
                conn.execute(f"DELETE FROM {table}")
                
            # reset database
            conn.execute("PRAGMA optimize")
            conn.commit()
        except:
            conn.rollback()
            raise

def rotation(q):
    q0, q1, q2, q3 = q
    S = np.array([
        [1 - 2*q2**2 - 2*q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), 1 - 2*q1**2 - 2*q3**2, 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*q1**2 - 2*q2**2]
    ])
    return S


if table_exists(conn, 'quadcopter_states'):
    truncate_all_tables(file_name)
    pass
else:
    sql_header_states = '''CREATE TABLE quadcopter_states
                    (Timestamp NUMBER,
                    X_axis_coordinate NUMBER,
                    Y_axis_coordinate NUMBER,
                    Z_axis_coordinate NUMBER,
                    Quaternion_w NUMBER,
                    Quaternion_x NUMBER,
                    Quaternion_y NUMBER,
                    Quaternion_z NUMBER,
                    Velocity_bx NUMBER,
                    Velocity_by NUMBER,
                    Velocity_bz NUMBER,
                    Angular_velocity_bx NUMBER,
                    Angular_velocity_by NUMBER,
                    Angular_velocity_bz NUMBER);'''
    cur.execute(sql_header_states)


    sql_header_inputs = '''CREATE TABLE quadcopter_inputs
                    (Timestamp NUMBER,
                    Motor1_rotational_speed NUMBER,
                    Motor2_rotational_speed NUMBER,
                    Motor3_rotational_speed NUMBER,
                    Motor4_rotational_speed NUMBER);'''
    cur.execute(sql_header_inputs)
    


# import mujoco
m = mujoco.MjModel.from_xml_path('./crazyfile/scene.xml')
d = mujoco.MjData(m)



g0  = 9.81     # [m.s^2] accerelation of gravity
mass  = 0.033    # [kg] total mass (with one marker)
Ixx = 1.395e-5   # [kg.m^2] Inertia moment around x-axis
Iyy = 1.395e-5   # [kg.m^2] Inertia moment around y-axis
Izz = 2.173e-5   # [kg.m^2] Inertia moment around z-axis
Cd  = 7.9379e-06 # [N/krpm^2] Drag coef
Ct  = 3.25e-4    # [N/krpm^2] Thrust coef
dq  = 65e-3      # [m] distance between motors' center

arm_length = dq/2.0  # The length of the motor lever arm (m)
max_thrust = 0.1573     # Maximum thrust of a single motor (N) (max = 22krpm)
max_torque = 3.842e-03  # Maximum torque of a single motor (Nm) (max = 22krpm)

# 100Hz 10ms 0.01s
dt = 0.01
#print(m.opt.timestep)
m.opt.timestep = dt
next_time = time.time() + dt
# m.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4

x0 = [0., 0., 0.1, 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.] # initial state
x0 = [0., 0., 0.1, 0.9924, 0.0868, 0.0868, 0.0076, 0., 0., 0., 0., 0., 0.]
d.qpos[0:7] = x0[0:7] # Initial Angle

# Time parameters
t0, t_end = 0, 3  # Time range (seconds)
dt = 0.01  # Time step (seconds)
N = int((t_end - t0) / dt)  # Number of steps
t_pts = np.linspace(t0, t_end, N + 1)

paused = False

def key_callback(keycode):
    if chr(keycode) == ' ':
        global paused
        paused = not paused

w = [21, 21, 21, 21]
d.actuator('motor1').ctrl[0] = calc_motor_input(w[0])
d.actuator('motor2').ctrl[0] = calc_motor_input(w[1])
d.actuator('motor3').ctrl[0] = calc_motor_input(w[2])
d.actuator('motor4').ctrl[0] = calc_motor_input(w[3])

sim_data = list()
sim_data.append(x0)

rk45 = list()
rk45.append(np.array(x0))

target = [0., 0., 0.5]

np.random.seed(42)
with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
    while viewer.is_running():
        for i in range(N):
            if not paused:
                
                w = pid_controller(control_callback(m, d), target)
                d.actuator('motor1').ctrl[0] = calc_motor_input(w[0])
                d.actuator('motor2').ctrl[0] = calc_motor_input(w[1])
                d.actuator('motor3').ctrl[0] = calc_motor_input(w[2])
                d.actuator('motor4').ctrl[0] = calc_motor_input(w[3])
                time.sleep(dt)
                '''
                sol_rk45 = solve_ivp(export_model, [0, dt], rk45[:][-1], args=(w,), method="RK45")
                rk45.append(sol_rk45.y[:, -1])'''
                #time.sleep(dt*10)
                mujoco.mj_step(m, d, nstep = 1)
                sim_data.append(np.add(control_callback(m, d), np.random.normal(0, 0.1, size=(13))))
                viewer.sync()
        break
'''
for i in range(N + 1):
    rk45[i][7:10] = rotation(rk45[i][3:7]) @ rk45[i][7:10]
rk45 = np.array(rk45)'''

# RK45
sol_rk45 = solve_ivp(export_model, [t0, t_end], x0, args=(w,), t_eval= t_pts, method="RK45")

v_rk45 = np.zeros([N + 1, 3], dtype=float)
for i in range(N + 1):
    v_rk45[i, :] = rotation(sol_rk45.y[3:7, i]) @ sol_rk45.y[7:10, i]


# RK23
sol_rk23 = solve_ivp(export_model, [t0, t_end], x0, args=(w,), t_eval= t_pts, method="RK23")

v_rk23 = np.zeros([N + 1, 3], dtype=float)
for i in range(N + 1):
    v_rk23[i, :] = rotation(sol_rk23.y[3:7, i]) @ sol_rk23.y[7:10, i]


# data storage
ws = np.full((N+1, 4), w)
#print(np.shape(ws))
timestamps = t_pts.reshape(N+1, 1)
#print(np.shape(timestamps))

sim_data = np.array(sim_data)
#print(np.shape(sim_data))

cur.executemany('INSERT INTO quadcopter_states VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)', 
                np.hstack([timestamps, sim_data]))
cur.executemany('INSERT INTO quadcopter_inputs VALUES (?,?,?,?,?)', 
                np.hstack([timestamps, ws]))


conn.commit()
cur.close()
conn.close()


# Redis
r = redis.Redis(host='localhost', port=6379, db=0)
pipe = r.pipeline()

def prepare_value(value):
    if isinstance(value, (np.int64, np.float64)):
        return value.item()  # 转换为Python原生类型
    return value

for i in range(N+1):
    timestamp = int(timestamps[i] * 1000) 
    key = f"drone:state:{timestamp}"
    
    # set Hash
    pipe.hset(key, mapping={
        "timestamp": prepare_value(timestamp),
        "motor1": prepare_value(ws[i, 0]),
        "motor2": prepare_value(ws[i, 1]),
        "motor3": prepare_value(ws[i, 2]),
        "motor4": prepare_value(ws[i, 3]),
        "pos_x": prepare_value(sim_data[i, 0]),
        "pos_y": prepare_value(sim_data[i, 1]),
        "pos_z": prepare_value(sim_data[i, 2]),
        "q_w": prepare_value(sim_data[i, 3]),
        "q_x": prepare_value(sim_data[i, 4]),
        "q_y": prepare_value(sim_data[i, 5]),
        "q_z": prepare_value(sim_data[i, 6]),
        "vel_x": prepare_value(sim_data[i, 7]),
        "vel_y": prepare_value(sim_data[i, 8]),
        "vel_z": prepare_value(sim_data[i, 9]),
        "ang_vel_x": prepare_value(sim_data[i, 10]),
        "ang_vel_y": prepare_value(sim_data[i, 11]),
        "ang_vel_z": prepare_value(sim_data[i, 12])
    })
    
    pipe.zadd("drone:timestamps", {timestamp: timestamp}) # sorted set
pipe.execute()
'''
for i in range(4):
    results = validate_dynamics(
        sim_data=np.vstack([sim_data[:, 3 + i], t_pts]),
        exp_data=np.vstack([sol_rk23.y[3 + i, :], t_pts]),
        time_vec=t_pts,
    )
    print("Max Errors:", results['max_error'])
    print("RMSE:", results['rmse'])
    print("Variance:", results['variance'])
'''
'''
# Plot
# Velocity
fig, ax = plt.subplots(3, 1)

lable = ['X', 'Y', 'Z']
for i in range(3):
    ax[i].plot(t_pts, v_rk45[:,i], "r-", label = "RK45")
    #ax[i].plot(t_pts, rk45[:, i+7], "k-", label = "RK45")
    ax[i].plot(t_pts, v_rk23[:,i], "g-", label = "RK23")
    ax[i].plot(t_pts, sim_data[:, 7+i], "b-", label = "simulation")
    ax[i].set(ylabel=f'{lable[i]}-axis direction velocity')
    ax[i].legend()
    ax[i].grid()

ax[2].set(xlabel = 'time/s')

# Position
fig, bx = plt.subplots(3, 1)
for i in range(3):
    bx[i].plot(t_pts, sol_rk45.y[i, :], "r-", label = "RK45")
    #bx[i].plot(t_pts, rk45[:, i], "k-", label = "RK45")
    bx[i].plot(t_pts, sol_rk23.y[i, :], "g-", label = "RK23")
    bx[i].plot(t_pts, sim_data[:, i], "b-", label = "simulation")
    bx[i].set(xlabel='time/s', ylabel=f'{lable[i]}-axis coordinate')
    bx[i].legend()
    bx[i].grid()
'''
'''
fig, cx = plt.subplots(1, 1)
header = ['x', 'y', 'z', 'q_w', 'q_x', 'q_y', 'q_z', 'v_x', 'v_y', 'v_z', 'w_x', 'w_y', 'w_z']
for i in range(13):
    cx.plot(t_pts, sim_data[:, i], "-", label = f'{header[i]}')
cx.set(xlabel='time/s', ylabel='Value of the states')
cx.legend()
cx.grid()
'''

# plt.tight_layout()
plt.show()
#print(sol_rk45.y[:, -1])
#print(sol_rk23.y[:, -1])
#print(sim_data[-1, :])
