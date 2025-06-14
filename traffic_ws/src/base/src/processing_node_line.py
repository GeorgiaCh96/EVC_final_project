#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

class LineTrackingViewNode:
    def __init__(self):
        rospy.init_node("line_tracking_view_node", xmlrpc_port=45600, tcpros_port=45601)
        self.bridge = CvBridge()

        self.sub_image = rospy.Subscriber(
            "/camera/image_undistorted", CompressedImage,
            self.image_cb, queue_size=1
        )
        self.pub_view = rospy.Publisher(
            "/camera/processed_image_line_tracking", CompressedImage,
            queue_size=1
        )

        # publish motion command to /motion/line_tracking_command topic
        self.command_pub = rospy.Publisher(
            "motion/line_tracking_command",
            String,
            queue_size=1)

        self.last_direction = "CENTERED"
        rospy.loginfo("Line tracking view node initialized.")

    def image_cb(self, msg):
        try:
            image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            height, width, _ = image.shape
            crop_start = int(height * 0.6)
            crop_left = int(width * 0.1)
            crop_right = int(width * 0.9)

            # Crop bottom portion and process (with reduced width)
            crop = image[crop_start:, crop_left:crop_right]
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)   # old threshold=100
            M = cv2.moments(thresh)
            annotated = image.copy()

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"]) + crop_left
                cy = int(M["m01"] / M["m00"]) + crop_start

                image_center = width // 2
                error = cx - image_center
                direction = "CENTERED"
                if abs(error) > 20:
                    direction = "LEFT" if error < 0 else "RIGHT"
                    if direction != self.last_direction:
                        self.command_pub.publish(String(data=direction))
                        self.last_direction = direction
                else:
                    self.last_direction = "CENTERED"
                    self.command_pub.publish(String(data=direction))

                # Visual annotations
                cv2.circle(annotated, (cx, cy), 10, (0, 0, 255), -1)
                cv2.line(annotated, (image_center, crop_start), (image_center, height), (255, 255, 0), 2)
                cv2.line(annotated, (cx, crop_start), (cx, height), (0, 255, 0), 2)
                cv2.putText(annotated, f"Offset: {error} px ({direction})", (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
            else:
                rospy.logwarn("[LineTracking] No line detected")
                cv2.putText(annotated, "No line detected", (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # Add threshold preview inset
            thresh_bgr = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
            inset = cv2.resize(thresh_bgr, (160, 120))
            annotated[10:130, -170:-10] = inset

            # Publish final annotated image
            success, encoded = cv2.imencode(".jpg", annotated)
            if success:
                out_msg = CompressedImage()
                out_msg.header.stamp = rospy.Time.now()
                out_msg.format = "jpeg"
                out_msg.data = encoded.tobytes()
                self.pub_view.publish(out_msg)

        except Exception as e:
            rospy.logerr("[LineTrackingView] Error: %s", str(e))

if __name__ == "__main__":
    try:
        LineTrackingViewNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
