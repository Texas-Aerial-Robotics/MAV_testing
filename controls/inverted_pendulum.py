import os
import matplotlib.pyplot as plt
import numpy as np
from control.matlab import ss
import control as ct

# System parameters
m = 0.5  # mass (kg)
l = 1.0  # length (m)
g = 9.8  # gravity (m/s^2)
b = 0.1  # damping coefficient (Ns/m)

# State-space matrices for the pendulum
A = [[0, 1],
     [-(g/l), -b/m]]
B = [[0],
     [1]]
C = [[1, 0]]
D = [[0]]

# Create the state-space system
sys = ss(A, B, C, D)

# Create figure with two subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# Time vector
t = np.linspace(0, 20, 1000)

# Initial conditions response
X0_1 = [np.pi/6, 0]  # First case
X0_2 = [np.pi/2, 0]  # Second case

# Simulate both initial conditions
t1, y1 = ct.initial_response(sys, T=t, X0=X0_1)
t2, y2 = ct.initial_response(sys, T=t, X0=X0_2)

# Plot initial condition responses
ax1.plot(t1, y1, label='Initial angle = pi/6', color='#2ecc71')
ax1.plot(t2, y2, label='Initial angle = pi/2', color='#e74c3c')
ax1.set_title('Initial Condition Response', pad=15)
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angle (rad)')
ax1.grid(True, linestyle='--', alpha=0.7)
ax1.legend()

# Create input signal for forced response (1 for t ≤ 2, 0 for t > 2)
T = t
U = np.where(t <= 2, 1, 0)
X0 = [0, 0]  # Initial conditions for forced response

# Simulate forced response
t_forced, y_forced = ct.forced_response(sys, T=T, U=U, X0=X0)

# Plot forced response
ax2.plot(t_forced, y_forced, label='System Response', color='#3498db')
ax2.plot(t, U, '--', label='Input u(t)', color='#e67e22', alpha=0.7)
ax2.set_title('Forced Response [Initial conditions: x(0)=0, dx/dt(0)=0]', pad=15)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angle (rad)')
ax2.grid(True, linestyle='--', alpha=0.7)
ax2.legend()

# Add text box with system parameters
param_text = f'Parameters:\nm = {m} kg\nl = {l} m\ng = {g} m/s^2\nb = {b} Ns/m'
ax1.text(0.02, 0.98, param_text, transform=ax1.transAxes,
         verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

plt.tight_layout()
plt.show()