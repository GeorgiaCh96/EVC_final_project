#!/usr/bin/env python2

import Jetson.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD) 

import rospy
from std_msgs.msg import String
from motors_lib.motorDriver import DaguWheelsDriver
from motors_lib.encoderDriver import WheelEncoderDriver
import math
import time

# Motor constants
WHEEL_RADIUS = 0.032
WHEEL_BASELINE = 0.18
TICKS_PER_REV = 137
GAIN = 12.603
TRIM = -0.093
GPIO_LEFT = 12
GPIO_RIGHT = 35

class FinalSubscriberNode:
    def __init__(self):
        rospy.init_node("final_subscriber_node")
        rospy.loginfo("Initializing Final Subscriber Node...")

        self.motor_driver = DaguWheelsDriver()
        self.encoder_left = WheelEncoderDriver(GPIO_LEFT)
        self.encoder_right = WheelEncoderDriver(GPIO_RIGHT)

        self.executing = False
        self.last_executed = None
        self.current_source = None

        self.priority_map = {
            'gesture': 3,
            'barcode': 2,
            'sign': 1
        }

        # Subscriptions
        rospy.Subscriber("/motion/gesture_motion_command", String, self.cb_factory('gesture'))
        rospy.Subscriber("/motion/barcode_motion_command", String, self.cb_factory('barcode'))
        rospy.Subscriber("/motion/sign_motion_command", String, self.cb_factory('sign'))

        rospy.on_shutdown(self.stop)
        rospy.loginfo("Final Subscriber Node ready.")

    def cb_factory(self, source):
        def callback(msg):
            command = msg.data.strip().upper()
            rospy.loginfo("Received '%s' command from %s" % (command, source))

            if self.current_source and self.priority_map[source] < self.priority_map[self.current_source]:
                rospy.loginfo("Ignoring %s command due to higher priority source: %s" %
                              (source, self.current_source))
                return

            self.current_source = source
            self.execute_command(command)

        return callback

    def stop(self):
        rospy.loginfo("Stopping robot...")
        self.motor_driver.set_wheels_speed(0, 0)

        if rospy.is_shutdown():
            rospy.loginfo("Cleaning up GPIO...")
            try:
                GPIO.cleanup()
            except Exception as e:
                rospy.logwarn("GPIO cleanup warning: %s" % str(e))

    def execute_command(self, cmd):
        if self.executing:
            rospy.loginfo("Already executing a command. Ignoring: %s" % cmd)
            return
        if cmd == self.last_executed:
            rospy.loginfo("Command already executed recently. Skipping: %s" % cmd)
            return

        self.executing = True
        try:
            if cmd == "STOP":
                self.stop()
            elif cmd == "GO":
                self.move_forward(0.2, 0.3)
            elif cmd == "TURN LEFT":
                self.turn(45, 0.3)
            elif cmd == "TURN RIGHT":
                self.turn(-45, 0.3)
            else:
                rospy.logwarn("Unknown command: %s" % cmd)
                self.stop()

            self.last_executed = cmd

        except Exception as e:
            rospy.logerr("Execution error: %s" % str(e))
        finally:
            self.executing = False

    def move_forward(self, dist, speed):
        rospy.loginfo("Moving forward: %.2f meters at speed %.2f" % (dist, speed))
        ticks = self.distance_to_ticks(dist)
        start_l = self.encoder_left._ticks
        start_r = self.encoder_right._ticks
        l, r = self.adjusted_speeds(speed)
        self.motor_driver.set_wheels_speed(l, r)

        while not rospy.is_shutdown():
            moved = (self.encoder_left._ticks - start_l + self.encoder_right._ticks - start_r) / 2.0
            if moved >= ticks:
                break
            time.sleep(0.01)

        self.stop()

    def turn(self, angle_deg, speed):
        rospy.loginfo("Turning %s: %.1f degrees at speed %.2f" %
                      ("LEFT" if angle_deg > 0 else "RIGHT", abs(angle_deg), speed))
        arc = (WHEEL_BASELINE * abs(math.radians(angle_deg))) / 2
        ticks = self.distance_to_ticks(arc)
        start_l = self.encoder_left._ticks
        start_r = self.encoder_right._ticks
        l, r = self.adjusted_speeds(speed)

        if angle_deg > 0:
            self.motor_driver.set_wheels_speed(l, -r)
        else:
            self.motor_driver.set_wheels_speed(-l, r)

        while not rospy.is_shutdown():
            moved = (abs(self.encoder_left._ticks - start_l) +
                     abs(self.encoder_right._ticks - start_r)) / 2.0
            if moved >= ticks:
                break
            time.sleep(0.01)

        self.stop()

    def distance_to_ticks(self, distance):
        revs = distance / (2 * math.pi * WHEEL_RADIUS)
        return revs * TICKS_PER_REV

    def adjusted_speeds(self, base_speed):
        left = base_speed * GAIN * (1 + TRIM)
        right = base_speed * GAIN * (1 - TRIM)
        return left, right


if __name__ == "__main__":
    try:
        FinalSubscriberNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
