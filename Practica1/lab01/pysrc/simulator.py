# Script to simulate a unicycle or integrator robot
import numpy as np
import scipy as sp
import robot as rb
from scipy import integrate

# TODO: Initial pose of the robot (define)
#p0 = np.array([np.float32(-7), np.float32(8)])
p0 = np.array([np.float32(8), np.float32(12),np.float32(3.14159)])
pf=[1,1]
# TODO: Robot object IntegratorRobot or UnicycleRobot (complete)
#rob = rb.IntegratorRobot(p0)
rob = rb.UnicycleRobot(p0,pf)

# Simulation parameter
tf = 20.0                # Simulation time
dt = 0.1                 # Simulation time step
steps = int(tf/dt) + 1   # Number of time steps

# Time steps to sample the trajectory
T = np.linspace(0, tf, steps)
# Trajectory of the robot (integral)
X = integrate.odeint(rob, p0, T)

# Create a GUI object to visualise the results
gui = rb.GraphicsRobot()

# Uncomment to visualise
# See trajetory animation
gui.AnimateTrajectory(T, X)

# See individual variables as a function of time
gui.PlotVariables(T, X)

# See the 2D trajectory of the robot with initial and final pose
gui.PlotTrajectory(X)