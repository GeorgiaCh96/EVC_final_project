#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

# CONSTANTS
LOWEST_PRIORITY = 10
COLOUR_PRIORITY = 4
GESTURE_PRIORITY = 2
BARCODE_PRIORITY = 3
YOLO_PRIORITY = 1
MOTION_PRIORITY = 0

class DecisionNode:
    def __init__(self):
        rospy.init_node("decision_node", xmlrpc_port=45700, tcpros_port=45701)
        rospy.loginfo("Initializing Decision Node......")

        # Initialize instance variables
        self.last_decision = None
        self.stopped = False
        self.current_priority = LOWEST_PRIORITY
        self.locked = False

        # Set subscriptions
        rospy.Subscriber("/motion/nomotion_motion_command", String, self.callback_wrapper(MOTION_PRIORITY))
        rospy.Subscriber("/motion/colour_motion_command", String, self.callback_wrapper(COLOUR_PRIORITY))
        rospy.Subscriber("/motion/gesture_motion_command", String, self.callback_wrapper(GESTURE_PRIORITY))
        rospy.Subscriber("/motion/barcode_motion_command", String, self.callback_wrapper(BARCODE_PRIORITY))
        rospy.Subscriber("/motion/yolo_motion_command", String, self.callback_wrapper(YOLO_PRIORITY))

        # Set publisher
        self.pub_decision = rospy.Publisher("/decision", String, queue_size=1)

        rospy.loginfo("Decision Node ready")

    def callback_wrapper(self, priority):
        def callback(msg):
            self.decide(msg, priority)
        return callback

    def decide(self, msg, priority):
        rospy.loginfo("Received command: %s [%d]" % (msg.data, priority))

        if priority > self.current_priority:
            rospy.logwarn("Too low priority: Ignoring %s", msg.data)
            return

        if msg.data == self.last_decision:
            rospy.logwarn("Previously executed: Ignoring %s", msg.data)
            return

        # Filter out lock
        if msg.data == "LOCK":
            if self.stopped and not self.locked:
                rospy.logwarn("Motion Detected: Locking Motors")
                self.current_priority = MOTION_PRIORITY
                self.locked = True
            return

        # Filter out unlock
        if msg.data == "UNLOCK":
            if self.stopped and self.locked:
                rospy.logwarn("No Motion Detected: Unlocking Motors")
                rospy.loginfo("Reseting priority")
                self.current_priority = LOWEST_PRIORITY
                self.locked = False
            return

        # Run command
        if not self.locked:
            self.pub_decision.publish(msg)
            self.last_decision = msg.data
            self.current_priority = priority

            # Resetting priority for stop
            if msg.data == "STOP":
                self.stopped = True
                rospy.loginfo("Reseting priority")
                self.current_priority = LOWEST_PRIORITY
            else:
                self.stopped = False

if __name__ == "__main__":
    try:
        DecisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
