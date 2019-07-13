#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math
from shared_utils.params import Params
from shared_utils.topics import Topics
from shared_utils.node_names import NodeNames
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

DBW_FREQUENCY = 50

class DBWNode(object):
    def __init__(self):
        rospy.init_node(NodeNames.DBW)

        self.steer_pub = Topics.Vehicle.SteeringCmd.Publisher(queue_size=1)
        self.throttle_pub = Topics.Vehicle.ThrottleCmd.Publisher(queue_size=1)
        self.brake_pub = Topics.Vehicle.BrakeCmd.Publisher(queue_size=1)

        self.controller = Controller(vehicle_mass=Params.DBW.VehicleMass.Get(),
                                    fuel_capacity=Params.DBW.FuelCapacity.Get(),
                                    brake_deadband=Params.DBW.BrakeDeadband.Get(),
                                    decel_limit=Params.DBW.DecelLimit.Get(),
                                    accel_limit=Params.DBW.AccelLimit.Get(),
                                    wheel_radius=Params.DBW.WheelRadius.Get(),
                                    wheel_base=Params.DBW.WheelBase.Get(),
                                    steer_ratio=Params.DBW.SteerRatio.Get(),
                                    max_lat_accel=Params.DBW.MaxLatAccel.Get(),
                                    max_steer_angle=Params.DBW.MaxSteerAngle.Get())

        # TODO: Subscribe to all the topics you need to
        Topics.Vehicle.DBWEnabled.Subscriber(self.dbw_enabled_cb, queue_size=1)
        Topics.TwistCmd.Subscriber(self.twist_cb, queue_size=2)
        Topics.CurrentVelocity.Subscriber(self.velocity_cb, queue_size=5)

        self.current_vel = None
        self.curr_ang_vel = None
        self.dbw_enabled = None
        self.linear_vel = None
        self.angular_vel = None
        self.throttle = self.steering = self.brake = 0

        self.loop()

    def loop(self):
        rate = rospy.Rate(DBW_FREQUENCY) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            if not None in (self.current_vel, self.linear_vel, self.angular_vel):
                self.throttle, self.brake, self.steering = self.controller.control(self.current_vel,
                                                                    self.dbw_enabled,
                                                                    self.linear_vel,
                                                                    self.angular_vel)

            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
            if self.dbw_enabled:
                self.publish(self.throttle, self.brake, self.steering)
            rate.sleep()


        self.loop()

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg

    def twist_cb(self, msg):
        self.linear_vel = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z

    def velocity_cb(self, msg):
        self.current_vel = msg.twist.linear.x

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
