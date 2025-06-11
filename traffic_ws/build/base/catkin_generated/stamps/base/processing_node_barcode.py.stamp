#!/usr/bin/env python2

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode

class BarcodeProcessingNode:
    def __init__(self):
        rospy.loginfo("Initializing barcode processing node...")
        rospy.init_node("processing_node_barcode", xmlrpc_port=45100, tcpros_port=45101)

        self.bridge = CvBridge()
        self.first_image_received = False

        # Subscribe to undistorted image
        self.sub_image = rospy.Subscriber(
            "/camera/image_undistorted",
            CompressedImage,
            self.image_cb,
            queue_size=1,
            buff_size=2**24
        )

        # Publish annotated image (for viewing)
        self.annotated_pub = rospy.Publisher("/camera/processed_image", CompressedImage, queue_size=1)

        # Publish QR result to command topic
        self.command_pub = rospy.Publisher("/motion/barcode_motion_command", String, queue_size=1)

        rospy.loginfo("Barcode processing node initialized!")

    def image_cb(self, msg):
        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("Barcode node received first image")

        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Decode barcodes / QR codes
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            decoded_objects = decode(gray)
            annotated_image = cv_image.copy()
            command_sent = False

            for obj in decoded_objects:
                try:
                    text = obj.data.decode("utf-8")
                except Exception as e:
                    rospy.logwarn("Decode error: %s" % str(e))
                    continue

                rospy.loginfo("Detected QR/Barcode: %s" % text)

                # Send to motion command
                self.command_pub.publish(String(text))
                command_sent = True

                # Annotate
                points = obj.polygon
                pts = np.array([[p.x, p.y] for p in points], np.int32)
                cv2.polylines(annotated_image, [pts], True, (0, 255, 0), 2)
                x, y = pts[0]
                cv2.putText(annotated_image, text, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            if not command_sent:
                rospy.loginfo("No QR code detected.")

            # Publish the annotated image
            success, encoded_image = cv2.imencode(".jpg", annotated_image)
            if success:
                out_msg = CompressedImage()
                out_msg.header.stamp = rospy.Time.now()
                out_msg.format = "jpeg"
                out_msg.data = encoded_image.tobytes()
                self.annotated_pub.publish(out_msg)

        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s" % str(e))
        except Exception as e:
            rospy.logerr("Unexpected error: %s" % str(e))

    def cleanup(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        node = BarcodeProcessingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
