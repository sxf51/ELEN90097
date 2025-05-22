import numpy as np

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

    # Position of the world coordinate system
    px = y[0]
    py = y[1]
    pz = y[2]
    # quaternion
    q0 = y[3]
    q1 = y[4]
    q2 = y[5]
    q3 = y[6]
    # World coordinate system velocity
    vx = y[7]
    vy = y[8]
    vz = y[9]
    # Angular velocity of the machine coordinate system
    wx = y[10]
    wy = y[11]
    wz = y[12]
    # state
    # x = [px, py, pz, q0, q1, q2, q3, vx, vy, vz, wx, wy, wz]

    # Input: The rotational speeds of four motors
    w1 = w[0]
    w2 = w[1]
    w3 = w[2]
    w4 = w[3]
    u = [w1, w2, w3, w4]

    # xdot = [px_dot, py_dot, pz_dot, q0_dot, q1_dot, q2_dot, q3_dot, vx_dot, vy_dot, vz_dot, wx_dot, wy_dot, wz_dot]

    S = np.array([
        [1 - 2*q2**2 - 2*q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), 1 - 2*q1**2 - 2*q3**2, 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*q1**2 - 2*q2**2]
    ])

    # Position differentiation
    [px_d, py_d, pz_d] = S @ [vx, vy, vz]

    # The derivative of velocity
    _thrust_acc_b = Ct*(w1**2 + w2**2 + w3**2 + w4**2) / mass  # The acceleration caused by thrust in the body coordinate system

    vx_d = - wx*vx - 2*(q1*q3 - q0*q2)*g0 - (0.0013*vx*abs(vx))/mass
    vy_d = - wy*vy - 2*(q0*q1 + q2*q3)*g0 - (0.0013*vy*abs(vy))/mass
    vz_d = _thrust_acc_b - wz*vz - 2*(0.5 - q1**2 - q2**2)*g0 - (0.00242*vz*abs(vz))/mass  # acceleration of gravity

    # The derivative of quaternions
    q0_d = -(q1*wx)/2 - (q2*wy)/2 - (q3*wz)/2
    q1_d =  (q0*wx)/2 - (q3*wy)/2 + (q2*wz)/2
    q2_d =  (q3*wx)/2 + (q0*wy)/2 - (q1*wz)/2
    q3_d = -(q2*wx)/2 + (q1*wy)/2 + (q0*wz)/2

    # moments
    mx = Ct*l*(  w1**2 - w2**2 - w3**2 + w4**2)
    my = Ct*l*( -w1**2 - w2**2 + w3**2 + w4**2)
    mz = Cd*  ( -w1**2 + w2**2 - w3**2 + w4**2)
    # The derivative of angular velocity
    wx_d = (mx + Iyy*wy*wz - Izz*wy*wz)/Ixx
    wy_d = (my - Ixx*wx*wz + Izz*wx*wz)/Iyy
    wz_d = (mz + Ixx*wx*wy - Iyy*wx*wy)/Izz

    # functions
    y_dot = [px_d, py_d, pz_d, q0_d, q1_d, q2_d, q3_d, vx_d, vy_d, vz_d, wx_d, wy_d, wz_d]

    return y_dot
