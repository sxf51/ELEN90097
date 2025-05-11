import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Use the linear equation (when the Angle is very small)
def export_model(t, y: list, w):
    # parameters
    g0  = 9.81     # [m.s^2] accerelation of gravity
    mass  = 0.033    # [kg] total mass (with one marker)
    Ixx = 1.395e-5   # [kg.m^2] Inertia moment around x-axis
    Iyy = 1.395e-5   # [kg.m^2] Inertia moment around y-axis
    Izz = 2.173e-5   # [kg.m^2] Inertia moment around z-axis
    Cd  = 7.9379e-06 # [N/krpm^2] Drag coef
    Ct  = 3.25e-4    # [N/krpm^2] Thrust coef
    dq  = 65e-3      # [m] distance between motors' center
    l   = dq/2       # [m] distance between motors' center and the axis of rotation

    # 世界坐标系位置  Position of the world coordinate system
    px = y[0]
    py = y[1]
    pz = y[2]
    # 四元数  quaternion
    q0 = y[3]
    q1 = y[4]
    q2 = y[5]
    q3 = y[6]
    # 世界坐标系速度  World coordinate system velocity
    vx = y[7]
    vy = y[8]
    vz = y[9]
    # 机体坐标系角速度  Angular velocity of the machine coordinate system
    wx = y[10]
    wy = y[11]
    wz = y[12]
    # 构建状态向量
    # x = [px, py, pz, q0, q1, q2, q3, vx, vy, vz, wx, wy, wz]

    # 系统控制输入: 四个电机的转速
    w1 = w[0]
    w2 = w[1]
    w3 = w[2]
    w4 = w[3]
    u = [w1, w2, w3, w4]

    # xdot = [px_dot, py_dot, pz_dot, q0_dot, q1_dot, q2_dot, q3_dot, vx_dot, vy_dot, vz_dot, wx_dot, wy_dot, wz_dot]

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
    vx_d = _thrust_accx_w - (0.00242 * vx * abs(vx))/mass
    vy_d = _thrust_accy_w - (0.00242 * vy * abs(vy))/mass
    vz_d = _thrust_accz_w - (0.00242 * vz * abs(vz))/mass - g0  # 重力加速度

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

    # functions
    # 构建表达式
    y_dot = [px_d, py_d, pz_d, q0_d, q1_d, q2_d, q3_d, vx_d, vy_d, vz_d, wx_d, wy_d, wz_d]

    return y_dot

'''
x0 = [0., 0., 0.1, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
# Time parameters
t0, t_end = 0, 0.1  # Time range (seconds)
dt = 0.01  # Time step (seconds)
N = int((t_end - t0) / dt)  # Number of steps
t_pts = np.linspace(t0, t_end, N + 1)

w = [22, 22, 22, 22]
solution = solve_ivp(export_model, [t0, t_end], x0, args=(w,), t_eval= t_pts, method="RK45")
'''
#print(solution.y[2, :])
'''
R = np.array([
        [1, 1,   1],
        [0,   2, 1],
        [0,   0,   3]
    ])

# 位置求导
px_d, py_d, pz_d = R @ [1, 2, 3]
print(px_d, py_d, pz_d)
'''