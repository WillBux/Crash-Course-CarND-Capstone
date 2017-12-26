#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
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

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.dbw_enabled = True
        self.lx_target = 0.0
        self.az_target = 0.0
        self.lx_current = 0.0
        self.az_current = 0.0
        self.current_angle = 0.0

        self.last_timestamp = rospy.rostime.get_time()

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        min_speed = 0.0
        self.controller = Controller(wheel_base=wheel_base,
                                     steer_ratio=steer_ratio,
                                     min_speed=min_speed,
                                     max_lat_accel=max_lat_accel,
                                     max_steer_angle=max_steer_angle,
                                     accel_limit=accel_limit,
                                     decel_limit=decel_limit,
                                     brake_deadband=brake_deadband,
                                     wheel_radius=wheel_radius,
                                     vehicle_mass=vehicle_mass)

        rospy.Subscriber('/current_velocity', TwistStamped, self.curr_vel_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_cb, queue_size=1)
        rospy.Subscriber('/vehicle/steering_report', SteeringReport, self.current_angle_cb, queue_size=1)

        # Open file to log commands
        # with open(self.logfile,"w") as f:
        #     f.write("Time, Lx-target, Lx-actual, Az-target, Az-actual, Throttle, Brake, Steer\n")

        self.loop()

    def dbw_cb(self, msg):
        rospy.loginfo("DBW STATUS = {}".format(msg))
        self.dbw_enabled = bool(msg.data)

    def curr_vel_cb(self, msg):
        self.lx_current = msg.twist.linear.x
        self.az_current = msg.twist.angular.z

    def twist_cmd_cb(self, msg):
        self.lx_target = math.fabs(msg.twist.linear.x)
        self.az_target = msg.twist.angular.z

    def current_angle_cb(self, msg):
        self.current_angle = msg.steering_wheel_angle_cmd

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():

            now = rospy.rostime.get_time()
            time_elapsed = now - self.last_timestamp
            self.last_timestamp = now

            params = {'lx_target': self.lx_target,
                      'az_target': self.az_target,
                      'lx_current': self.lx_current,
                      'az_current': self.az_current,
                      'dbw_enabled': self.dbw_enabled,
                      'current_steering_angle': self.current_angle,
                      'time_elapsed': time_elapsed}

            throttle, brake, steering = self.controller.control(**params)

            if self.dbw_enabled:
                self.publish(throttle, brake, steering)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        # if throttle > .0:
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)
        # elif brake > .0:
        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

if __name__ == '__main__':
    DBWNode()
