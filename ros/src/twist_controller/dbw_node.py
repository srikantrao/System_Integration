#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        self.simulation = rospy.get_param('~simulation', False)

        ## Initialize all velocities
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.current_velocity = 0
        self.current_angular_velocity = 0
        self.dbw_is_on = False

        # Asynchronous publish with queue size of 1
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        # self.controller = Controller(self.vehicle_mass, self.fuel_capacity,self.brake_deadband,
        #                              self.decel_limit, self.accel_limit, self.wheel_radius,
        #                              self.wheel_base, self.steer_ratio, self.max_lat_accel,self.max_steer_angle)

        self.controller = Controller(self.simulation)   # To check whether it is simulation or on actual site 

        # TODO: Subscribe to all the topics you need to

        # Subscribe to the /twist_cmd topic --> Get linear_velocity and angular_velocity from here
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)

        # Subscribe to the /vehicle/dbw_enabled topic --> get self.dwb_is_on from here
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        # Subscribe to the /current_velocity topic --> get current_velocity  and current_angular_velocity from here
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)


        self.loop()

    def twist_cb(self, msg):
        """
        Extract expected linear and angular velocity
        :param msg: TwistStamped message based on rosmsg info
        :return: Side effect is that linear_velocity and angular velocity gets updated
        """
        # In vehicle co-ordinates, so only need X
        self.linear_velocity = msg.twist.linear.x

        # Yaw is in the Z axis --> Ignore Roll and Pitch
        self.angular_velocity = msg.twist.angular.z

        #rospy.logwarn("Linear Velocity: %.2f, Angular Velocity: %2f", self.linear_velocity, self.angular_velocity)

    def current_velocity_cb(self, msg):
        """
        Extract the current velocity of the vehicle
        :param msg: TwistStamped message based on rosmsg info
        :return: Side effect is that self.current_velocity gets updated
        """
        # Get the current linear velocity
        self.current_velocity = msg.twist.linear.x

        # Get the current angular velocity
        self.current_angular_velocity = msg.twist.angular.z
        #rospy.logwarn("Current Velocity: %2f", self.current_velocity)


    def dbw_enabled_cb(self, msg):
        """
        Callback to check if the Drive by wire is enabled or not
        """
        self.dbw_is_on = bool(msg.data)


    def loop(self):

        # Reducing the rate to try and get the simulator working
        rate = rospy.Rate(50)

        # Time step - 1/50 = 0.02
        deltaT = 0.02

        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            throttle, brake, steer = self.controller.control(self.linear_velocity, self.angular_velocity,
                                                                self.current_velocity, deltaT, self.dbw_is_on)

            #rospy.logwarn("throttle:%.2f, brake:%.2f, steer:%.2f", throttle, brake, steer)

            if self.dbw_is_on:
                self.publish(throttle, brake, steer)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

if __name__ == '__main__':
    DBWNode()
