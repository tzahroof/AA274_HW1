import numpy as np
import math
from numpy import linalg
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt
from utils import *

# Constants
tf = 15
V_max = 0.5
om_max = 1

# time
dt = 0.005
N = int(tf/dt)
t = dt*np.array(range(N+1)) 

# Initial conditions
x_0 = 0
y_0 = 0
V_0 = V_max
th_0 = -np.pi/2
xd_0 = V_0*np.cos(th_0)
yd_0 = V_0*np.sin(th_0)

# Final conditions
x_f = 5
y_f = 5
V_f = V_max
th_f = -np.pi/2
xd_f = V_f*np.cos(th_f)
yd_f = V_f*np.sin(th_f)


def differential_flatness_trajectory():
    '''
    This function solves a system of equations and computes the state trajectory.
    and control history (V (t), om(t)). 
    Outputs:
        traj: a numpy array of size [T, state_dim] where T is the number of time steps, and state_dim is 6. 
        The state ordering needs to be [x,y,th,xd,yd,xdd,ydd]
    
    HINT: You may find the function linalg.solve useful
    '''
    ########## Code starts here ##########

    # Solve for xi coefficients

    A = np.array([[1, 0, 0 , 0],
        [0, 1, 0 , 0],
        [1, tf, tf**2, tf**3],
        [0, 1, 2*tf, 3*(tf**2)]])

    bx = np.array([x_0, V_0*np.cos(th_0), x_f, V_f*np.cos(th_f)]);
    by = np.array([y_0, V_0*np.sin(th_0), y_f, V_f*np.sin(th_f)]);
    x_sol = np.linalg.solve(A, bx)
    y_sol = np.linalg.solve(A, by)

    # Determine traj

    x_traj = x_sol[0] + x_sol[1]*t + x_sol[2]*t**2 + x_sol[3]*t**3;
    y_traj = y_sol[0] + y_sol[1]*t + y_sol[2]*t**2 + y_sol[3]*t**3;

    xd_traj = x_sol[1] + 2*x_sol[2] * t + 3 * x_sol[3]*t**2;
    yd_traj = y_sol[1] + 2*y_sol[2] * t + 3 * y_sol[3]*t**2;

    xdd_traj = 2*x_sol[2] + 6*x_sol[3]*t;
    ydd_traj = 2*y_sol[2] + 6*y_sol[3]*t;

    theta_traj = np.arctan2(yd_traj,xd_traj);

    traj = np.concatenate((np.vstack(x_traj),
            np.vstack(y_traj),
            np.vstack(theta_traj),
            np.vstack(xd_traj),
            np.vstack(yd_traj),
            np.vstack(xdd_traj),
            np.vstack(ydd_traj)), axis=1);

    # Determine control inputs from states

    V = np.zeros(t.shape[0])
    om = np.zeros(t.shape[0])

    V[0] = V_0
    om[0] = 0

    for i in range(1,t.shape[0]):
        V[i] = np.sqrt(xd_traj[i]**2 + yd_traj[i]**2)
        om[i] = (theta_traj[i] - theta_traj[i-1])/dt

    ########## Code ends here ##########
    return traj, V, om

def compute_arc_length(V, t):
    '''
    This function computes arc-length s as a function of t.
    Inputs:
        V: a vector of velocities of length T
        t: a vector of time of length T
    Output:
        s: the arc-length as a function of time. s[i] is the arc-length at time t[i]. This has length T.
    HINT: Use the function cumtrapz
    HINT: This should take one line
    '''
    ########## Code starts here ##########

    s = cumtrapz(V, t, initial=0)

    ########## Code ends here ##########
    return s

def rescale_V(V, om):
    '''
    This function computes V_tilde, given the unconstrained solution V, and om.
    Inputs:
        V: vector of velocities of length T. Solution from the unconstrained, differential flatness problem.
        om:  vector of angular velocities of length T. Solution from the unconstrained, differential flatness problem.
    Output:
        V_tilde: Rescaled velocity that satisfies the control constraints.

    HINT: at each timestep V_tilde should be computed as a minimum of the original value V, 
        and values required to ensure _both_ constraints are satisfied

    HINT: This should take one line
    '''
    ########## Code starts here ##########
    V_tilde = np.copy(V)

    for i in range(V.shape[0]):
        V_tilde[i] = min(V_max, V[i])
        if(om[i] != 0):
            V_tilde[i] = min(V_tilde[i], abs(V[i]/om[i]))

    ########## Code ends here ##########

    return V_tilde


def compute_tau(V_tilde, s):
    '''
    This function computes the new time history tau as a function of s.
    Inputs:
        V_tilde: a vector of scaled velocities of length T.
        s: a vector of arc-length of length T.
    Output:
        tau: the new time history as a function of time. tau[i] is the time at s[i]. This has length T.
    HINT: Use the function cumtrapz
    HINT: This should take one line
    '''
    ########## Code starts here ##########

    tau = cumtrapz(1/V_tilde, s, initial=0)

    ########## Code ends here ##########
    return tau

def rescale_om(V, om, V_tilde):
    '''
    This function computes the rescaled om control
    Inputs:
        V: vector of velocities of length T. Solution from the unconstrained, differential flatness problem.
        om:  vector of angular velocities of length T. Solution from the unconstrained, differential flatness problem.
        V_tilde: vector of scaled velocities of length T.
    Output:
        om_tilde: vector of scaled angular velocities
    HINT: This should take one line.
    '''
    ########## Code starts here ##########


    om_tilde = V_tilde/V * om

    ########## Code ends here ##########
    return om_tilde


if __name__ == "__main__":
    traj, V, om = differential_flatness_trajectory()
    s = compute_arc_length(V, t)
    V_tilde = rescale_V(V, om)
    tau = compute_tau(V_tilde, s)
    om_tilde = rescale_om(V, om, V_tilde)

    # Get new final time
    tf_new = tau[-1]

    # Generate new uniform time grid
    N_new = int(tf_new/dt)
    t_new = dt*np.array(range(N_new+1))
    t_new = t_new.T

    # Interpolate for state trajectory
    data_scaled = np.zeros((N_new+1,7))
    data_scaled[:,0] = np.interp(t_new,tau,traj[:,0]) # x
    data_scaled[:,1] = np.interp(t_new,tau,traj[:,1]) # y
    data_scaled[:,2] = np.interp(t_new,tau,traj[:,2]) # th
    # Interpolate for scaled velocities
    V_scaled = np.interp(t_new, tau, V_tilde)                # V
    om_scaled = np.interp(t_new, tau, om_tilde)              # om
    # Compute xy velocities
    data_scaled[:,3] = V_scaled*np.cos(data_scaled[:,2]) # xd
    data_scaled[:,4] = V_scaled*np.sin(data_scaled[:,2]) # yd
    # Compute xy acclerations
    data_scaled[:,5] = np.append(np.diff(data_scaled[:,3])/dt,-V_f*om_scaled[-1]*np.sin(th_f)) # xdd
    data_scaled[:,6] = np.append(np.diff(data_scaled[:,4])/dt, V_f*om_scaled[-1]*np.cos(th_f)) # ydd

    # Save trajectory data
    data = {'z': data_scaled, 'V': V_scaled, 'om': om_scaled}
    save_dict(data, "data/differential_flatness.pkl")
    maybe_makedirs('plots')
    
    # Plots
    plt.figure(figsize=(15, 7))
    plt.subplot(2, 2, 1)
    plt.plot(data_scaled[:,0], data_scaled[:,1], 'k-',linewidth=2)
    plt.grid('on')
    plt.plot(x_0, y_0, 'go', markerfacecolor='green', markersize=15)
    plt.plot(x_f, y_f, 'ro', markerfacecolor='red', markersize=15)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title("Path (position)")
    plt.axis([-1, 6, -1, 6])

    plt.subplot(2, 2, 2)
    plt.plot(t, V, linewidth=2)
    plt.plot(t, om, linewidth=2)
    plt.grid('on')
    plt.xlabel('Time [s]')
    plt.legend(['V [m/s]', '$\omega$ [rad/s]'], loc="best")
    plt.title('Original Control Input')
    plt.tight_layout()

    plt.subplot(2, 2, 4)
    plt.plot(t_new, V_scaled, linewidth=2)
    plt.plot(t_new, om_scaled, linewidth=2)
    plt.grid('on')
    plt.xlabel('Time [s]')
    plt.legend(['V [m/s]', '$\omega$ [rad/s]'], loc="best")
    plt.title('Scaled Control Input')
    plt.tight_layout()

    plt.subplot(2, 2, 3)
    plt.plot(t, s, 'b-', linewidth=2)
    plt.grid('on')
    plt.plot(tau, s, 'r-', linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Arc-length [m]')
    plt.legend(['Original', 'Scaled'], loc="best")
    plt.title('Original and scaled arc-length')
    plt.tight_layout()
    plt.savefig("plots/differential_flatness.png")