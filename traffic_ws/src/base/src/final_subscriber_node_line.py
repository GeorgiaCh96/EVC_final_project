#!/usr/bin/env python2

import Jetson.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD) 

import rospy
from std_msgs.msg import String, Bool
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
        rospy.loginfo("Initializing Final Subscriber Node......")

        self.motor_driver = DaguWheelsDriver()
        self.encoder_left = WheelEncoderDriver(GPIO_LEFT)
        self.encoder_right = WheelEncoderDriver(GPIO_RIGHT)
        self.motion_lock = False
        self.initialized = True
        self.line_tracking_enabled = False
        self.last_line_direction = "CENTERED"
        self.tracking_timer = rospy.Timer(rospy.Duration(0.2), self.tracking_update)


        # Set subscriber
        rospy.Subscriber("/decision", String, self.deserialize)
        rospy.Subscriber("/motion/line_tracking_command", String, self.line_tracking)

        
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Final Subscriber Node ready.")

    def stop(self):
        rospy.loginfo("Stopping robot...")
        self.motor_driver.set_wheels_speed(0, 0)
        self.line_tracking_enabled = False
    
    def shutdown(self):
        rospy.loginfo("Shutting down robot...")
        self.motor_driver.set_wheels_speed(0, 0)

        if rospy.is_shutdown():
            rospy.loginfo("Cleaning up GPIO...")
            try:
                GPIO.cleanup()
            except Exception as e:
                rospy.logwarn("GPIO cleanup warning: %s" % str(e))

    def deserialize(self, cmd):
        command = cmd.data.split()
        rospy.loginfo("Deserializing %s", command[0])
        
        try:
            if len(command) == 0:
                raise Exception("Received empty command")
            elif command[0] == "STOP":
                self.stop()
            else:
                if command[0] == "GO":
                    self.move_forward(float(command[1]))
                elif command[0] == "TURN":
                    self.turn(float(command[1]), float(command[2]))
        except Exception as e:
            rospy.logerr("Execution error: %s" % str(e))
            self.stop()

    def line_tracking(self, msg):
        if not self.line_tracking_enabled:
            return
        direction = msg.data.upper()
        self.last_line_direction = direction
        rospy.loginfo("[LineTracking] Direction update: %s", direction)

    def tracking_update(self, event):
        if self.line_tracking_enabled and hasattr(self, 'base_speed'):
            self.move_forward(self.base_speed)



    def move_forward(self, speed):
        rospy.loginfo("Moving forward: at speed %.2f" % (speed))
        self.line_tracking_enabled = True
        self.base_speed = speed
        l_base, r_base = self.adjusted_speeds(speed)
        l, r = l_base, r_base

            # Apply corrections based on last known direction
        if self.last_line_direction == "RIGHT":
            r = 0.6 * r_base  # reduce left speed slightly
            l = 0.9 * l_base
            rospy.loginfo("Correcting LEFT")
        elif self.last_line_direction == "LEFT":
            l = 0.6 * l_base  # reduce right speed slightly
            r = 0.9 * r_base
            rospy.loginfo("Correcting RIGHT")
        else:
            rospy.loginfo("CENTERED no correction")
            l, r = l_base, r_base

        self.motor_driver.set_wheels_speed(l, r)


    def turn(self, angle_deg, speed):
        rospy.loginfo("Turning %s: %.1f degrees at speed %.2f" %
                      ("LEFT" if angle_deg > 0 else "RIGHT", abs(angle_deg), speed))
        arc = (WHEEL_BASELINE * abs(math.radians(angle_deg))) / 2
        ticks = self.distance_to_ticks(arc)
        start_l = self.encoder_left._ticks
        start_r = self.encoder_right._ticks
        l, r = self.adjusted_speeds(speed)
        self.line_tracking_enabled = False

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
            
        self.move_forward(0.02)

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
