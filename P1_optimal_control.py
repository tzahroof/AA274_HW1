import numpy as np
import math
import scikits.bvp_solver
import matplotlib.pyplot as plt
from utils import *

dt = 0.005

def ode_fun(tau, z):
    '''
    This function computes the dz given tau and z. It is used in the bvp solver.
    Inputs:
        tau: the independent variable. This must be the first argument.
        z: the state vector. The first three states are [x, y, th, ...]
    Output:
        dz: the state derivative vector. Returns a numpy array.
    '''
    ########## Code starts here ##########

    x = z[0];
    y = z[1];
    theta = z[2];
    p1 = z[3];
    p2 = z[4];
    p3 = z[5];
    r = z[6];

    V = -(p1*np.cos(theta) + p2*np.sin(theta))/2;
    w = -p3/2;

    dz = np.array([r*V*np.cos(theta), 
        r*V*np.sin(theta),
        r*w,
        0,
        0,
        r*(p1*V*np.sin(theta) - p2*V*np.cos(theta)),#possible error
        0]);

    ########## Code ends here ##########
    return dz


def bc_fun(za, zb):
    '''
    This function computes boundary conditions. It is used in the bvp solver.
    Inputs:
        za: the state vector at the initial time
        zb: the state vector at the final time
    Output:
        bca: tuple of boundary conditions at initial time
        bcb: tuple of boundary conditions at final time
    '''
    # final goal pose
    xf = 5
    yf = 5
    thf = -np.pi/2.0
    xf = [xf, yf, thf]
    # initial pose
    x0 = [0, 0, -np.pi/2.0]

    ########## Code starts here ##########

    lambda_var = 2.0 #Adjust this

    theta = zb[2]
    p1 = zb[3]
    p2 = zb[4]
    p3 = zb[5]


    V = -(p1*np.cos(theta) + p2*np.sin(theta))/2;
    w = -p3/2

    #TODO: check if this H is correct
    H = lambda_var + V**2 + w**2 + p1*V*np.cos(theta) +p2*V*np.sin(theta) + p3*w


    bca = np.array([za[0]-x0[0], za[1]-x0[1], za[2]-x0[2]]);
    bcb = np.array([zb[0]-xf[0], zb[1]-xf[1], zb[2]-xf[2], H]);


    ########## Code ends here ##########
    return (bca, bcb)

def solve_bvp(problem_inputs, initial_guess):
    '''
    This function solves the bvp_problem.
    Inputs:
        problem_inputs: a dictionary of the arguments needs to define the problem
                        num_ODE, num_parameters, num_left_boundary_conditions, boundary_points, function, boundary_conditions
        initial_guess: initial guess of the solution
    Outputs:
        z: a numpy array of the solution. It is of size [time, state_dim]

    Read this documentation -- https://pythonhosted.org/scikits.bvp_solver/tutorial.html

    '''
    problem = scikits.bvp_solver.ProblemDefinition(**problem_inputs)
    soln = scikits.bvp_solver.solve(problem, solution_guess=initial_guess)

    # Test if time is reversed in bvp_solver solution
    flip, tf = check_flip(soln(0))
    t = np.arange(0,tf,dt)
    z = soln(t/tf)
    if flip:
        z[3:7,:] = -z[3:7,:]
    z = z.T # solution arranged so that it is [time, state_dim]
    return z

def compute_controls(z):
    '''
    This function computes the controls V, om, given the state z. It is used in main().
    Input:
        z: z is the state vector for multiple time instances. It has size [time, state_dim]
    Outputs:
        V: velocity control input
        om: angular rate control input
    '''
    ########## Code starts here ##########

    #TODO: note that time is rows, z state is in columns
    theta = z[:,2]
    p1 = z[:,3]
    p2 = z[:,4]
    p3 = z[:,5]

    V = -(p1*np.cos(theta) + p2*np.sin(theta))/2;
    # V = -(np.multiply(p1 , np.cos(theta)) + np.multiply(p2 , np.sin(theta)) )/2

    #om = -p3/2
    om = -p3/2;
    ########## Code ends here ##########

    return V, om

def main():
    '''
    This function solves the specified bvp problem and returns the corresponding optimal contol sequence
    Outputs:
        V: optimal V control sequence 
        om: optimal om control sequence
    You are required to define the problem inputs, initial guess, and compute the controls

    HINT: The total time is between 15-25
    '''
    ########## Code starts here ##########


    initial_guess = (2.0,2.0, -np.pi/2.0, -2, -2, 0.5, 20)
    num_ODE = 7;
    num_parameters = 0;
    num_left_boundary_conditions = 3;
    boundary_points = (0,1)
    function = ode_fun
    boundary_conditions = bc_fun



    ########## Code ends here ##########

    problem_inputs = {
                      'num_ODE' : num_ODE,
                      'num_parameters' : num_parameters,
                      'num_left_boundary_conditions' : num_left_boundary_conditions,
                      'boundary_points' : boundary_points,
                      'function' : function,
                      'boundary_conditions' : boundary_conditions
                     }

    z = solve_bvp(problem_inputs, initial_guess)
    V, om = compute_controls(z)
    return z, V, om

if __name__ == '__main__':
    z, V, om = main()
    tf = z[0,-1]
    t = np.arange(0,tf,dt)
    x = z[:,0]
    y = z[:,1]
    th = z[:,2]
    data = {'z': z, 'V': V, 'om': om}
    save_dict(data, 'data/optimal_control.pkl')
    maybe_makedirs('plots')

    # plotting
    # plt.rc('font', weight='bold', size=16)
    plt.figure(figsize=(12, 4))
    plt.subplot(1, 2, 1)
    plt.plot(x, y,'k-',linewidth=2)
    plt.quiver(x[1:-1:200], y[1:-1:200],np.cos(th[1:-1:200]),np.sin(th[1:-1:200]))
    plt.grid('on')
    plt.plot(0,0,'go',markerfacecolor='green',markersize=15)
    plt.plot(5,5,'ro',markerfacecolor='red', markersize=15)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis([-1, 6, -1, 6])
    plt.title('Optimal Control Trajectory')

    plt.subplot(1, 2, 2)
    plt.plot(t, V,linewidth=2)
    plt.plot(t, om,linewidth=2)
    plt.grid('on')
    plt.xlabel('Time [s]')
    plt.legend(['V [m/s]', '$\omega$ [rad/s]'], loc='best')
    plt.title('Optimal control sequence')
    plt.tight_layout()
    plt.savefig('plots/optimal_control.png')
    plt.show()
