from yaw_controller import YawController
from lowpass import LowPassFilter
import numpy as np
from pid import PID
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args):
        """
        Controller Class to calculate throttle, brake and steer
        :param args:
        :param kwargs:
        """
        self.simulation = args[0]

        # If it true then set all parameters to simulation values
        if self.simulation:
            self.vehicle_mass = 1080
            self.fuel_capacity = 0
            self.brake_deadband = 0.2
            self.decel_limit = -5
            self.accel_limit = 1
            self.wheel_radius = 0.335
            self.wheel_base = 3
            self.steer_ratio = 14.8
            self.max_lat_accel = 3.
            self.max_steer_angle = 8
        else:
            self.vehicle_mass = 1736.35
            self.fuel_capacity = 13.5
            self.brake_deadband = 0.1
            self.decel_limit = -1
            self.accel_limit = 1
            self.wheel_radius = 0.2413
            self.wheel_base = 2.8498
            self.steer_ratio = 14.8
            self.max_lat_accel = 3.
            self.max_steer_angle = 8


        # Use PID control for throttle and brake -
        # if throttle is negative then use brake, else brake remains 0
        Kp = 0.1
        Ki = 0.0001
        Kd = 0.05

        # Throttle and Brake control
        self.throttle_pid = PID(Kp, Ki, Kd, self.decel_limit, self.accel_limit)
        # Use filtering ( exp window of 5 )
        self.brake_filter = LowPassFilter(0.25, 0.75)
        self.throttle_filter = LowPassFilter(0.5, 0.5)
        self.velocity_error = 0

        #Steering Control
        self.prev_steer = 0
        if self.simulation:
            # Using low pass filter or Exponentiall Weighted Averages - average over 25
            self.steer_filter = LowPassFilter(0.05, 0.95)
        else:
            # Using low pass filter or Exponentiall Weighted Averages - average over 10
            self.steer_filter = LowPassFilter(0.1, 0.9)
    def control(self, linear_velocity, angular_velocity, current_velocity, deltaT, dbw_is_on):
        """
        :param args: At a minimum will need the linear_velocity, angular_velocity and current_velocity.
        Look at yaw_controller for more details about each param.
        :param kwargs:
        :return:
        """
        throttle = steer = brake = 0
        # TODO: Change the arg, kwarg list to suit your needs
        if dbw_is_on:

            brake = 0

            # Use YawController to calculate the steering angle
            yc = YawController(self.wheel_base, self.steer_ratio, 0, self.max_lat_accel, self.max_steer_angle)

            # Calculate the steering angle - Start off with just simple calculation
            steer = yc.get_steering(linear_velocity, angular_velocity, current_velocity)

            # Use the LowPass filtering function here
            steer = self.steer_filter.filt(steer)

            #Update the prev_steer value
            self.prev_steer =steer

            #rospy.logwarn("Steering Angle :%.2f", steer)

            rospy.logwarn("Expected Velocity :%.2f", linear_velocity)

            # Use the throttle PID to figure out the next
            velocity_error = linear_velocity - current_velocity
            throttle = self.throttle_pid.step(velocity_error, deltaT)

            self.velocity_error = velocity_error

            # If throttle is negative, then brake needs to be activated
            if throttle < 0:
                brake = -throttle
                throttle = 0
                # Filter the brake values
                brake = self.brake_filter.filt(brake)
                if brake > 0.1:
                    brake = 0.1
                rospy.logwarn("Car is braking Brake:%.2f", brake)
            else:
                # Filter Throttle value
                throttle = self.throttle_filter.filt(throttle)

            # Looser bound for running simulation
            if self.simulation:
                if throttle > 0.2:
                    throttle = 0.2
                    brake = 0
            else:
            # Bound it based on ROSBAG output for actual car
                if throttle > 0.01:
                    throttle = 0.01
                    brake = 0

        else:
            # Control is now with the driver, so reset the PID controller
            self.throttle_pid.reset()

        rospy.logwarn("Velocity Error :%.2f, Throttle: %.2f", self.velocity_error, throttle)

        return throttle, brake, steer
