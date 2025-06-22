# ELEN90097 Project

The file tree is as follows:

``` bash
.
|   data_redis.ipynb # Analyze data using redis
|   data_sqlite.ipynb # Analyze data using sqlite
|   draw.py # Generate the animation of the formula
|   main.py # Using the state space equation, MuJoCo simulation, data storage, ODE solution, and visualization.
|   
+---crazyfile
|   |   cf2.png # Quadcopter picture
|   |   cf2.xml # Quadcopter model and parameter definition
|   |   scene.xml # Visualization of the quadrotor model
|   |   
|   \---assets # Quadcopter model file
|           
+---data # Data and database file
|       50Hertz.csv
|       Amprion.csv
|       combined_data.csv
|       datasets.py
|       quadcopter.db
|       TenneTTSO.csv
|       TransnetBW.csv
|       
\---src
        controller.py # Callback function of MuJoCo and the PID control algorithm.
        dynamic.py # Dynamic equation
        validate.py # Error analysis and verification
```

**Quadcopter unmanned aerial vehicle: mathematical modeling, simulation, data storage and analysis.**

## Parameters

```python
    g0  = 9.8066     # [m.s^2] accerelation of gravity
    mq  = 33e-3      # [kg] total mass (with one marker)
    Ixx = 1.395e-5   # [kg.m^2] Inertia moment around x-axis
    Iyy = 1.395e-5   # [kg.m^2] Inertia moment around y-axis
    Izz = 2.173e-5   # [kg.m^2] Inertia moment around z-axis
    Cd  = 7.9379e-06 # [N/krpm^2] Drag coef
    Ct  = 3.25e-4    # [N/krpm^2] Thrust coef
    dq  = 65e-3      # [m] distance between motors' center
    l   = dq/2       # [m] distance between motors' center and the axis of rotation

```

Motor speed range: 0-22krpm

Maximum tensile force of a single motor:

$$
F_{max}=C_t\cdot\omega_{max}^2=3.25\times10^{-4}\times22^2=0.1573N
$$

Maximum reverse torque of a single motor:

$$
M_{max}=C_d\cdot\omega_{max}^2=7.9379\times10^{-6}\times22^2=3.842\times10^{-3} N\cdot m
$$

In Mujoco, we use normalized inputs, and the input range for all motors is 0 to 1.

```xml
  <actuator>
    <motor class="cf2" ctrlrange="0 1" gear="0 0 0.1573 0 0 -3.842e-03" site="motor1_site" name="motor1"/>
    <motor class="cf2" ctrlrange="0 1" gear="0 0 0.1573 0 0 3.842e-03" site="motor2_site" name="motor2"/>
    <motor class="cf2" ctrlrange="0 1" gear="0 0 0.1573 0 0 -3.842e-03" site="motor3_site" name="motor3"/>
    <motor class="cf2" ctrlrange="0 1" gear="0 0 0.1573 0 0 3.842e-03" site="motor4_site" name="motor4"/>
  </actuator>
```

## Calculation

Calculate the rotation matrix for converting from the body coordinate system to the world coordinate system based on quaternions:

$$
R_b^w=
\begin{bmatrix}
1-2q_2^2-2q_3^2  & 2(q_1\cdot q_2-q_0\cdot q_3) & 2(q_1\cdot q_3 + q_0\cdot q_2) \\
2(q_1\cdot q_2 + q_0\cdot q_3)  & 1-2q_1^2-2q_3^2 & 2(q_2\cdot q_3 - q_0\cdot q_1)\\
2(q_1\cdot q_3 - q_0\cdot q_2)  & 2(q_2\cdot q_3 + q_0\cdot q_1) & 1-2q_1^2-2q_2^2
\end{bmatrix}
$$

### Quaternion differentiation

In the case of small-angle changes, the minor changes in quaternions can be expressed as follows:

$$
\Delta q=
\begin{bmatrix}
1 \\ \frac{\Delta \theta}{2}
\end{bmatrix}
$$

$$
\Delta \theta = \boldsymbol \omega\cdot dt
$$

Update the quaternion：

$$
q\otimes \Delta q= \frac{1}{2}
\begin{bmatrix}
0  & -\Delta \theta_x & -\Delta \theta_y & -\Delta \theta_z\\
\Delta \theta_x  & 0 & \Delta \theta_z & -\Delta \theta_y\\
\Delta \theta_y  & -\Delta \theta_z & 0 & \Delta \theta_x\\
\Delta \theta_z  & \Delta \theta_y & -\Delta \theta_x & 0
\end{bmatrix}
$$

Quaternion approximate differentiation:

$$
q{}' = \frac{1}{2}
\begin{bmatrix}
 0 & -\omega_x & -\omega_y & -\omega_z\\
\omega_x  & 0 & \omega_z & -\omega_y\\
\omega_y  & -\omega_z & 0 & \omega_x\\
\omega_z  & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

Program：

```python
    dq0 = -(q1*wx)/2 - (q2*wy)/2 - (q3*wz)/2
    dq1 =  (q0*wx)/2 - (q3*wy)/2 + (q2*wz)/2
    dq2 =  (q3*wx)/2 + (q0*wy)/2 - (q1*wz)/2
    dq3 =  (q1*wy)/2 - (q2*wx)/2 + (q0*wz)/2
```

### The derivative of angular velocity

M is the torque input.

$$
\mathrm {M}=\mathrm{I}\dot{\boldsymbol \omega} +\boldsymbol \omega\times(\mathrm{I}\boldsymbol \omega)
$$

Thus, the formula for the differential of angular velocity, that is, angular acceleration, is obtained:

Program：

```python
dwx = (mx + Iyy*wy*wz - Izz*wy*wz)/Ixx
dwy = (my - Ixx*wx*wz + Izz*wx*wz)/Iyy
dwz = (mz + Ixx*wx*wy - Iyy*wx*wy)/Izz
```

Where the torque is：

```python
mx = Ct*l*(  w1**2 - w2**2 - w3**2 + w4**2)
my = Ct*l*( -w1**2 - w2**2 + w3**2 + w4**2)
mz = Cd*  ( -w1**2 + w2**2 - w3**2 + w4**2)
```

1. Motor1 and Motor4 make the body rotate in the forward direction around its own X-axis, while Motor2 and Motor3 make the body rotate in the reverse direction around its own X-axis

2. Motor1 and Motor2 make the body rotate in the reverse direction around its own Y-axis. Motor3 and Motor4 make the body rotate in the forward direction around its own Y-axis

3. Motor1 and Motor3 produce clockwise reverse twisting. Motor2 and Motor4 produce counterclockwise reverse twisting (counterclockwise is positive).

### Position differentiation

Is the speed

### Velocity derivation

The overall thrust direction of the four motors of the quadcopter is along the Z-axis of the body coordinate system. Convert the thrust to the world coordinate system and add the gravitational force.

## State vector

Using a 13-dimensional state vector:

```python
x = [px, py, pz, q0, q1, q2, q3, vx, vy, vz, wx, wy, wz]
```

px, py, pz: Position of the world coordinate system

q0, q1, q2, q3: quaternion

vx, vy, vz: World coordinate system velocity

wx, wy, wz: Angular velocity of the body coordinate system

Use a 4-dimensional input vector, that is, the input of the rotational speeds of four motors, with the unit of krpm:

```python
u = [w1, w2, w3, w4]
```

## SQLite

For SQL, two entities are defined: inputs and states. In these two tables, timestamps serve as the primary keys. Other attributes contain duplicate values and thus cannot be utilized as foreign keys.

| Entity             | Attributes | Primary Key | Foreign Key | Constraints |
|--------------------|---------------------------|-------------|-------------|------------------|
| quadcopter\_inputs | Timestamp                 | Timestamp   | -           | NOT NULL         |
|                    | Motor1\_rotational\_speed |             |             | CHECK($>0, <22$) |
|                    | Motor2\_rotational\_speed |             |             | CHECK($>0, <22$) |
|                    | Motor3\_rotational\_speed |             |             | CHECK($>0, <22$) |
|                    | Motor4\_rotational\_speed |             |             | CHECK($>0, <22$) |
| quadcopter\_states | Timestamp                 | Timestamp   | -           | NOT NULL         |
|                    | X\_axis\_coordinate       |             |             | NOT NULL         |
|                    | Y\_axis\_coordinate       |             |             | NOT NULL         |
|                    | Z\_axis\_coordinate       |             |             | NOT NULL         |
|                    | Quaternion\_w             |             |             | CHECK($>-1, <1$) |
|                    | Quaternion\_x             |             |             | CHECK($>-1, <1$) |
|                    | Quaternion\_y             |             |             | CHECK($>-1, <1$) |
|                    | Quaternion\_z             |             |             | CHECK($>-1, <1$) |
|                    | Velocity\_bx              |             |             | NOT NULL         |
|                    | Velocity\_by              |             |             | NOT NULL         |
|                    | Velocity\_bz              |             |             | NOT NULL         |
|                    | Angular\_velocity\_bx     |             |             | NOT NULL         |
|                    | Angular\_velocity\_by     |             |             | NOT NULL         |
|                    | Angular\_velocity\_bz     |             |             | NOT NULL         |

## Redis

In the context of NoSQL and Redis, hash tables are employed to store status data. This approach has the advantage of maintaining a clear field structure. Meanwhile, Sorted sets are utilized to store the time index, which enables efficient range queries. In Redis, the function of the Key bears resemblance to that of the primary key in SQL. Consequently, each data entry adopts 'drone:state:{timestamp}' as the unique key (integer type, measured in milliseconds).

| Entity   | Attribute | Data Type   | Redis Storage Example |
|------------------|--------------------|----------------------|--------------------------------|
| drone:state      | timestamp          | Unix timestamp (int) | HSET timestamp 10              |
| ~                | motor1             | Float                | HSET ... motor1 10             |
| ~                | motor2             | Float                | HSET ... motor2 10             |
| ~                | motor3             | Float                | HSET ... motor3 10             |
| ~                | motor4             | Float                | HSET ... motor4 10             |
| ~                | pos\_x             | Float                | HSET ... pos\_x 0.1            |
| ~                | pos\_y             | Float                | HSET ... pos\_y 0.2`           |
| ~                | pos\_z             | Float                | HSET ... pos\_z 0.3`           |
| ~                | q\_w               | Float                | HSET ... q\_w 1                |
| ~                | q\_x               | Float                | HSET ... q\_x 0                |
| ~                | q\_y               | Float                | HSET ... q\_y 0                |
| ~                | q\_z               | Float                | HSET ... q\_z 0                |
| ~                | vel\_x             | Float                | HSET ... vel\_x 0.01           |
| ~                | vel\_y             | Float                | HSET ... vel\_y 0.02           |
| ~                | vel\_z             | Float                | HSET ... vel\_z 0.03           |
| ~                | ang\_vel\_x        | Float                | HSET ... ang\_vel\_x 0.001     |
| ~                | ang\_vel\_y        | Float                | HSET ... ang\_vel\_y 0.002     |
| ~                | ang\_vel\_z        | Float                | HSET ... ang\_vel\_z 0.003     |
| drone:timestamps | (Sorted Set)       | Unix timestamp       | ZADD drone:timestamps 10 10    |
