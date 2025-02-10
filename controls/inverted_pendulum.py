import os
import matplotlib.pyplot as plt
import numpy as np
from control.matlab import ss, step
import control as ct

# Set up better plotting style
plt.style.use('seaborn-v0_8-darkgrid')
plt.rcParams.update({
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 14,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'figure.figsize': (15, 6)
})

# System parameters
m = 0.5  # mass (kg)
l = 1.0  # length (m)
g = 9.8  # gravity (m/s^2)
b = 0.1  # damping coefficient (N⋅s/m)

# State-space matrices for the pendulum
A = [[0, 1],
     [-(g/l), -b/m]]  # Note the negative sign for g/l
B = [[0],
     [1]]
C = [[1, 0]]
D = [[0]]

# Create the state-space system
sys = ss(A, B, C, D)

# Create figure
plt.figure(figsize=(12, 5))

# Initial conditions
X0_1 = [np.pi/6, 0]  # First case: x(0) = π/6, ẋ(0) = 0
X0_2 = [np.pi/2, 0]  # Second case: x(0) = π/2, ẋ(0) = 0

# Time vector
t = np.linspace(0, 20, 1000)

# Simulate both initial conditions
t1, y1 = ct.initial_response(sys, T=t, X0=X0_1)
t2, y2 = ct.initial_response(sys, T=t, X0=X0_2)

# Plot responses
plt.plot(t1, y1, label='x(0) = π/6', color='#2ecc71')
plt.plot(t2, y2, label='x(0) = π/2', color='#e74c3c')

# Configure plot
plt.title('Pendulum Response to Different Initial Conditions', pad=15)
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend()

# Add text box with system parameters
param_text = f'm = {m} kg\nl = {l} m\ng = {g} m/s²\nb = {b} N⋅s/m'
plt.text(0.02, 0.98, param_text, transform=plt.gca().transAxes,
         verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

plt.tight_layout()

# Save or display the figure
if 'CONTROL_PLOT_DIR' in os.environ:
    plt.savefig(
        os.path.join(os.environ['CONTROL_PLOT_DIR'], 'pendulum_initial_conditions.pdf'),
        bbox_inches='tight',
        dpi=300
    )
else:
    plt.show()