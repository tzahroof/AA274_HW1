import numpy as np
from numpy import linalg
from P3_pose_stabilization import ctrl_pose

def ctrl_traj(x, y, th,
              ctrl_prev,
              x_d, y_d,
              xd_d, yd_d,
              xdd_d, ydd_d,
              x_g, y_g, th_g):
    '''
    This function computes the closed-loop control law.
    Inputs:
        (x,y,th): current state
        ctrl_prev: previous control input (V,om)
        (x_d, y_d): desired position
        (xd_d, yd_d): desired velocity
        (xdd_d, ydd_d): desired acceleration
        (x_g,y_g,th_g): desired final state
    Outputs:
        (V, om): a numpy array np.array([V, om]) containing the desired control inputs
    '''

    # Timestep
    dt = 0.005
    
    ########## Code starts here ##########

    kpx = 1
    kpy = 1
    kdx = 2
    kdy = 2

    dist_from_goal = np.sqrt((x_g - x)**2 + (y_g - y)**2)

    if (dist_from_goal < 0.5):
      # switch control laws because we're close to the goal
      [V, om] = ctrl_pose(x,y,th,x_g,y_g,th_g)
    else:

      # Set up and solve for a, w
      V_prev = ctrl_prev[0]
      w_prev = ctrl_prev[1]

      xdot = V_prev*np.cos(th)
      ydot = V_prev*np.sin(th)

      u1 = xdd_d + kpx *(x_d - x) + kdx * (xd_d - xdot)
      u2 = ydd_d + kpy *(y_d - y) + kdy * (yd_d - ydot)

      J = np.array([[np.cos(th) , -V_prev*np.sin(th)],
                         [np.sin(th) , V_prev*np.cos(th)]])

      u = np.array([u1, u2])

      aw = linalg.solve(J, u)

      a = aw[0]
      om = aw[1]


      # Integrate to obtain V
      V = V_prev + a * dt

      #reset V if it becomes 0 to avoid singularity
      if(abs(V) < 0.01):
        V = np.sqrt((xd_d - xdot)**2 + (yd_d - xdot)**2)

      #Apply saturation limits
      V = np.sign(V) * min(0.5, abs(V))
      om = np.sin(om) * min(1.0, abs(om))

    ########## Code ends here ##########

    return np.array([V, om])