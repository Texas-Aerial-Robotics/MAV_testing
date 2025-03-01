import numpy as np
import matplotlib.pyplot as plt

def S(t):
    # Step response for tau=0.01, gamma=2:
    # S(t) = 2 * (1 - e^(-100 t)) for t>=0, else 0
    return 2*(1 - np.exp(-100*t)) * np.heaviside(t, 0)

def x_of_t(t):
    # Input jumps: +2 at 0, then -1 at 0.05, +4 at 0.10, -2 at 0.101, -3 at 0.15
    forced_part = (
          2*S(t)
        - 1*S(t-0.05)
        + 4*S(t-0.10)
        - 2*S(t-0.101)
        - 3*S(t-0.15)
    )
    # Homogeneous part to enforce x(0)=2:
    hom_part = 2*np.exp(-100*t)
    return forced_part + hom_part

t_vals = np.linspace(0, 0.2, 1000)  # just plot up to 0.2
x_vals = x_of_t(t_vals)

plt.figure()
plt.plot(t_vals, x_vals)
plt.title("Fast response (tau=0.01, gamma=2)")
plt.xlabel("Time [s]")
plt.ylabel("x(t)")
plt.grid(True)
plt.show()
