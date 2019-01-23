#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float32MultiArray, String
import tf
import numpy as np
from numpy import linalg
from utils import wrapToPi


class Controller:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
	rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def gazebo_callback(self, data):
        pose = data.pose[data.name.index("turtlebot3_burger")]
        twist = data.twist[data.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def get_ctrl_output(self):
    # use self.x self.y and self.theta to 
	# compute the right control input here
    	
        # Code copied and pasted from Problem 3.
        
        #Following lines added to add state/goals for turtlebot
        xg = 1.0
        yg = 1.0
        thg = 0.0

        x = self.x
        y = self.y
        th = self.theta

        # Copy + Pasted code starts here
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

        # Following line commented out; no longer necessary
        # ctrl = np.array([V,om])

        ########## Code ends here ##########


        # More lines to convert V, om -> cmd_x_dot and cmd_theta_dot
        cmd_x_dot = V*np.cos(th)
        cmd_theta_dot = om

    	### END OF YOUR CODE ###        

        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot
        return cmd

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            ctrl_output = self.get_ctrl_output()
            self.pub.publish(ctrl_output)
            rate.sleep()

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
