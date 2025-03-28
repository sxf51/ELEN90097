'''
import numpy as np 
import scipy.integrate as spi 
import matplotlib.pyplot as plt 

R = 1
L = 1
C = 1

A = np.array([[1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0]])

B = np.array([[0, 0, 0, 1/C, 0],
            [1, 1, 1, 0, 0],
            [0, 0, -1, R, 0],
            [0, 1/L, 0, 1, 0],
            [0, 0, 0, 1, -1]])

C = np.array([[0],[-1],[0],[0],[0]])

def func(t, y):
    Vac = np.sin(t)
    return B @ y

N = 40
y0 = [0,0,0,0,0]
t_span = [0, N]
t_eval = np.linspace(0, N, 1000)

sol = spi.solve_ivp(func, y0= y0, t_span= t_span, t_eval= t_eval)

# We plot the particle's trajectory.
fig, ax = plt.subplots()
ax.plot(solution.y[0,:],solution.y[1,:],'o-')
ax.set(xlabel='Location',ylabel='Height', title='Trajectory')
ax.set_xlim(0, 5)
ax.set_ylim(0, 3)
'''

import numpy as np
from scipy.integrate import solve_ivp

def lotkavolterra(t, z, a, b, c, d):
    x, y = z
    return [a*x - b*x*y, -c*y + d*x*y]

sol = solve_ivp(lotkavolterra, [0, 15], [10, 5], args=(1.5, 1, 3, 1),
                dense_output=True)

t = np.linspace(0, 15, 300)
z = sol.sol(t)
import matplotlib.pyplot as plt
plt.plot(t, z.T)
plt.xlabel('t')
plt.legend(['x', 'y'], shadow=True)
plt.title('Lotka-Volterra System')
plt.show()
