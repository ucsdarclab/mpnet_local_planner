#!/usr/bin/python
# A pure pursuit controller for mobile robot

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_srvs.srv import Empty

def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
        

class Controller:
    def __init__(self):
        self.v = 0.4
        self.wheelbase = 0.325
        self.max_steering = np.pi/2 # 0.34
        self.state = np.array([0,0,0])
        self.state_hat = np.array([0,0,0])
        self.global_goal = np.array([0,0,0])
        self.path = []

        # Tuning parameters
        self.K = 2.5
        self.look_ahead = 0.075

        self.ackermann_cmd_topic = 'drive'
        self.frame_id = 'odom'
        rospy.Subscriber('/odom', Odometry, self.observe)
        # self.local_plan = '/rrt_path'
        self.local_plan = '/move_base/MpnetLocalPlanner/local_plan'
        rospy.Subscriber(self.local_plan, Path, self.callback_path)
        self.drive_pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=1)
        self.target_pub = rospy.Publisher('target_point', PointStamped, queue_size=1)
        # Objects to record data
        self.error_ld = []
        self.error_theta = []
        self.steering_angle = []
        self.target_point = []
        self.current_point = []

    def get_position(self, data):
        '''
        Converts input position and orientation into SE(3) co-ordinates
        :param pose: An object of type PoseStamped
        :returns numpy.array : Robot position in SE(3) co-ordinates
        '''
        q = data.pose.orientation
        _, _, yaw = euler_from_quaternion ([q.x, q.y, q.z, q.w]) 
        return np.array([data.pose.position.x, data.pose.position.y, yaw])

    def observe(self, data):
        '''
        A callback function that sets the state of the robot
        '''
        # print(data)
        self.state = self.get_position(data.pose)

    def callback_path(self, path):
        '''
        A callback function to get the published path
        '''
        self.path = [self.get_position(pose) for pose in path.poses]

    def prune_path(self):
        '''
        Removes old goal points from path
        '''
        index_bound, = np.where([np.linalg.norm(self.state[:2]-point[:2])<=self.look_ahead for point in self.path])
        if len(self.path)>1 and len(index_bound)>0:
            self.path = self.path[int(index_bound[-1]):]
        elif len(self.path)>1:
            self.path = self.path[1:]
            

    def set_goal(self):
        if len(self.path)==0:
            self.state_hat = self.state
        else:
            self.prune_path()
            self.state_hat = self.path[0]

    def get_msg(self, steering_angle, speed):
        '''
        A function to return AckermannDriveStamped message
        :param steering_angle : The steering angle of the car
        :param speed : The speed of the car
        :returns AckermannDriveStamped: An object to control the car with
        '''
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        return msg

    def getPointStamped(self, x, y ):
        '''
        Returns an object of type PointStamped
        :param x: x co-ordinate of the robot
        :param y: y co-ordinate of the robot
        :returns PointStamped: An object of type PointStamped
        '''
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.point.x = x
        msg.point.y = y
        return msg

    def reached_goal(self):
        '''
        A function to check if the robot has reached its goal
        :returns Bool: True if the robot has reached its goal
        '''
        if len(self.path)>=1:
            return np.linalg.norm(self.state[:2]-self.path[-1][:2])<0.1
        return True

    def control(self):
        # print(self.state, self.state_hat)
        msgTargetPoint = self.getPointStamped(self.state_hat[0], self.state_hat[1])
        self.target_pub.publish(msgTargetPoint)
        angle = pi_2_pi(self.state_hat[2] - self.state[2])
        ld = np.linalg.norm(self.state[:2]-self.state_hat[:2])
        steering = np.arctan2(2 * self.wheelbase * np.sin(angle / 2), self.v * 1) + self.K*angle
        steering = 0.0 if np.isclose(steering, 0.0) else steering  # Need to change this inside simulation code
        steering_angle = np.clip(steering, -self.max_steering, self.max_steering)
        speed = self.v # if ld > 0.2 else 0
        msg = self.get_msg(steering_angle, speed)
        if self.reached_goal():
            self.drive_pub.publish(self.get_msg(0.0,0.0))
        else:
            # print(msg.drive.steering_angle, msg.drive.speed)
            self.drive_pub.publish(msg)
            # Record data
            self.error_ld.append(ld)
            self.error_theta.append(angle)
            self.steering_angle.append(steering_angle)
            self.target_point.append(self.state_hat)
            self.current_point.append(self.state)
    
    def reset_planner(self, req):
        '''
        Empties the path buffer of the planner
        '''
        self.path = []
        msg = self.get_msg(0.0, 0.0)
        self.drive_pub.publish(msg)
        return []

    def save_data(self):
        '''
        A hook to save useful data for analysis
        '''
        msg = self.get_msg(0.0, 0.0)
        self.drive_pub.publish(msg)
        # np.save('/root/data/controller/error_d.npy',self.error_ld)
        # np.save('/root/data/controller/error_theta.npy', self.error_theta)
        # np.save('/root/data/controller/steering_angle.npy', self.steering_angle)
        # np.save('/root/data/controller/target_point.npy', self.target_point)
        # np.save('/root/data/controller/current_point.npy', self.current_point)

    
if __name__ =="__main__":
    rospy.init_node('pure_pursuit')
    c = Controller()
    reset_controller = rospy.Service('reset_controller', Empty, c.reset_planner)
    r = rospy.Rate(5) # 10hz
    rospy.on_shutdown(c.save_data)
    while not rospy.is_shutdown():
        c.set_goal() 
        c.control()
        r.sleep()