import numpy as np
import mujoco
import mujoco.minimize as minimize

sim_model = mujoco.MjModel.from_xml_path("./cartmodel.xml")
sim_data = mujoco.MjData(sim_model)

sim_model.body("cart")

# we can change the properties of the cart, for example mass as below
sim_model.body("cart").mass=np.array([1.0])

# This is a trick to list objects in the model
try:
    sim_model.body()
except KeyError as e:
    print(e)

# random forcing and measure data# Reset data, set initial pose.
mujoco.mj_resetData(sim_model, sim_data)
identification_data = {
    "qhist": [],
    "vhist": [],
    "dvhist": [],
    "uhist": [],}
for i in range(3000):
    identification_data["qhist"].append(sim_data.qpos.copy())
    identification_data["vhist"].append(sim_data.qvel.copy())
    u = np.random.randn(1) * 10
    identification_data["uhist"].append(u)
    sim_data.ctrl = u
    mujoco.mj_step(sim_model, sim_data)
    identification_data["dvhist"].append(sim_data.qacc.copy())
identification_data["qhist"] = np.array(identification_data["qhist"])
identification_data["vhist"] = np.array(identification_data["vhist"])
identification_data["dvhist"] = np.array(identification_data["dvhist"])
identification_data["uhist"] = np.array(identification_data["uhist"])

# formulate regressor
A = []
b = []
g = 9.81
for i in range(1, len(identification_data["qhist"])):
    q = identification_data["qhist"][i]
    v = identification_data["vhist"][i]
    dv = identification_data["dvhist"][i]
    u = identification_data["uhist"][i]
    regressor = [
        [dv[0], -dv[1] * np.cos(q[1]) + v[1] ** 2 * np.sin(q[1]), 0],
        [0, 0, dv[1]],
    ]
    A.extend(regressor)
    b.extend(
        [
            u[0],
            dv[0] * np.cos(q[1]) + g * np.sin(q[1]),
        ]
    )
A = np.array(A)
b = np.array(b)

def residual(x):
    # Ensure x is 2D for consistent processing
    if x.ndim == 1:
        x = x[:, np.newaxis]
    # Compute residuals using matrix operations
    return A @ x - b[:, np.newaxis]

result = minimize.least_squares(
    np.array((0.0, 0.0, 0.0)),
    residual,)
identified_parameters = result[0]
newL = identified_parameters[2]
new_poleM = identified_parameters[1] / newL
new_cartM = identified_parameters[0] - new_poleM

identified_parameters