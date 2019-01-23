import numpy as np
from utils import wrapToPi
import math

def ctrl_pose(x, y, th, xg, yg, thg):
    '''
    This function implements the pose stabilization controller.
    Inputs:
        x, y, th: the current pose of the robot
        xg, yg, thg: the desired pose of the robot
    Outputs:
        ctrl: a numpy array np.array([V, om]) containing the desired control inputs
    HINT: you need to use the wrapToPi function
    HINT: don't forget to saturate your control inputs
    '''

    ########## Code starts here ##########

    rho = np.sqrt((xg-x) ** 2 + (yg-y) ** 2)
    alpha = wrapToPi(np.arctan2(yg-y,xg-x)-(th))
    delta = wrapToPi(alpha + th - thg)


    k1 = 0.8
    k2 = 0.1
    k3 = 0.5

    V = k1 * rho * np.cos(alpha)
    om = k2 * alpha + k1 * np.sinc(alpha/np.pi) * np.cos(alpha) * (alpha + k3 * delta)

    # saturation limits
    V = np.sign(V) * min(abs(V),0.5)
    om = np.sign(om) * min(abs(om),1)

    ctrl = np.array([V,om])

    ########## Code ends here ##########

    return np.array(ctrl)