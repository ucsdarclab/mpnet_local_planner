#!/usr/bin/python3
# A Subscriber for twist message

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class RobotCommand():
    def __init__(self):
        self.pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size = 10)
        self.sub = rospy.Subscriber('/mpc_cmd_vel', Twist, self.callback)
        self.rate = rospy.Rate(100) # 10Hz

    def callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        theta_dot = msg.angular.z
        d = 0.324
        # TODO : Need to update the steering angle
        speed = np.hypot(theta_dot*d, np.hypot(vx,vy))
        action = AckermannDriveStamped()
        theta = np.arctan2(d*theta_dot, np.hypot(vx,vy))
        action.drive.steering_angle = theta
        action.drive.speed = speed
        self.pub.publish(action)
        self.rate.sleep()
    
if __name__=="__main__":
    rospy.init_node("bridge")
    robot_command = RobotCommand()
    rospy.spin()
