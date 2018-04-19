#!/usr/bin/env python
import rospy
import sys

import numpy as np
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from math import cos, sin, asin, tan, atan2
# msgs and srv for working with the set_model_service
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty

# a handy tool to convert orientations
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class BasicThymio:

    def __init__(self, thymio_name):
        """init"""
        self.thymio_name = thymio_name
        rospy.init_node('basic_thymio_controller', anonymous=True)

        # Publish to the topic '/thymioX/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(self.thymio_name + '/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber(self.thymio_name + '/odom',
                                                Odometry, self.update_state)

        self.current_pose = Pose()
        self.current_twist = Twist()
        # publish at this rate
        self.rate = rospy.Rate(10)

    def thymio_state_service_request(self, position, orientation):
        """Request the service (set thymio state values) exposed by
        the simulated thymio. A teleportation tool, by default in gazebo world frame.
        Be aware, this does not mean a reset (e.g. odometry values)."""
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            model_state = ModelState()
            model_state.model_name = self.thymio_name
            model_state.reference_frame = '' # the frame for the pose information
            model_state.pose.position.x = position[0]
            model_state.pose.position.y = position[1]
            model_state.pose.position.z = position[2]
            qto = quaternion_from_euler(orientation[0], orientation[0], orientation[0], axes='sxyz')
            model_state.pose.orientation.x = qto[0]
            model_state.pose.orientation.y = qto[1]
            model_state.pose.orientation.z = qto[2]
            model_state.pose.orientation.w = qto[3]
            # a Twist can also be set but not recomended to do it in a service
            gms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            response = gms(model_state)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def update_state(self, data):
        """A new Odometry message has arrived. See Odometry msg definition."""
        # Note: Odmetry message also provides covariance
        self.current_pose = data.pose.pose
        self.current_twist = data.twist.twist
        quat = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion (quat)
        rospy.loginfo("State from Odom: (%.5f, %.5f, %.5f) " % (self.current_pose.position.x, self.current_pose.position.y, yaw))

    def basic_move(self):
        """Moves the migthy thymio"""
        vel_msg = Twist()
        vel_msg.linear.x = 0.2 # m/s
        vel_msg.angular.z = 0. # rad/s

        while not rospy.is_shutdown():
            # Publishing thymo vel_msg
            self.velocity_publisher.publish(vel_msg)
            # .. at the desired rate.
            self.rate.sleep()

        # Stop thymio. With is_shutdown condition we do not reach this point.
        #vel_msg.linear.x = 0.
        #vel_msg.angular.z = 0.
        #self.velocity_publisher.publish(vel_msg)

        # waiting until shutdown flag (e.g. ctrl+c)
        rospy.spin()

def usage():
    return "Wrong number of parameters. basic_move.py [thymio_name]"

if __name__ == '__main__':
    if len(sys.argv) == 2:
        thymio_name = sys.argv[1]
        print "Now working with robot: %s" % thymio_name
    else:
        print usage()
        sys.exit(1)
    thymio = BasicThymio(thymio_name)

    # Teleport the robot to a certain pose. If pose is different to the
    # origin of the world, you must account for a transformation between
    # odom and gazebo world frames.
    # NOTE: The goal of this step is *only* to show the available
    # tools. The launch file process should take care of initializing
    # the simulation and spawning the respective models
    
    #thymio.thymio_state_service_request([0.,0.,0.], [0.,0.,0.])
    #rospy.sleep(1.)

    thymio.basic_move()
