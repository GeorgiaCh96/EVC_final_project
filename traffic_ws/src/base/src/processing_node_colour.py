#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
import webcolors
from scipy.spatial import KDTree

class ColorNamer:
    def __init__(self):
        self.names = webcolors.names("css3")  # List of color names
        self.rgb_values = []

        for name in self.names:
            hex_code = webcolors.name_to_hex(name)
            self.rgb_values.append(webcolors.hex_to_rgb(hex_code))

        # Store values in a k-d tree
        self.kdtree = KDTree(self.rgb_values)

    # Get the name of the colour for an rgb triplet
    def name_from_rgb(self, rgb_tuple):
        try:
            return webcolors.rgb_to_name(rgb_tuple)
        except ValueError:
            _, idx = self.kdtree.query(rgb_tuple)
            return self.names[idx]

class CameraProcessingNode:
    def __init__(self):
        rospy.loginfo("Initializing camera processing node...")
        self.bridge = CvBridge()
        self.color_namer = ColorNamer()

        self.sub_image = rospy.Subscriber(
            "/camera/image_undistorted",
            CompressedImage,
            self.image_cb,
            queue_size=1,
            buff_size=2**24
        )

        # self.processed_image_pub = rospy.Publisher("/camera/processed_image_colour", CompressedImage, queue_size=1)
        # self.hex_pub = rospy.Publisher("/camera/dominant_color_hex", String, queue_size=1)
        self.pub_image = rospy.Publisher(
            "/camera/processed_image_colour",
            CompressedImage,
            queue_size=1
        )

        self.colour_pub = rospy.Publisher(
            "/motion/colour_motion_command",
            String,
            queue_size=1)


        self.first_image_received = False
        rospy.loginfo("Camera processing node initialized.")

    def image_cb(self, data):
        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("Camera subscriber captured first image.")

        try:
            # Decode and resize image
            cv_image = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)

            hex_color, bgr = self.get_dominant_color(cv_image)
            rgb = (bgr[2], bgr[1], bgr[0])
            color_name = self.color_namer.name_from_rgb(rgb)

            # Annotate image
            # cv2.rectangle(cv_image, (0, 0), (50, 50), bgr, -1)
            # label = f"{hex_color} {color_name}"
            # cv2.putText(cv_image, label, (60, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2)
            label = f"{hex_color} {color_name}"

            move = self.decide_movement(color_name)
            if move:
                self.colour_pub.publish(move)
            else:
                pass

            cv2.rectangle(cv_image, (0, 0), (250, 50), bgr, -1)
            cv2.putText(cv_image, label, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)

            # cv2.rectangle(cv_image, (0, 0), (100, 40), bgr, -1)
            # cv2.putText(cv_image, label, (110, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            success, encoded_image = cv2.imencode(".jpg", cv_image)
            if success:
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                msg.data = encoded_image.tobytes()
                # self.processed_image_pub.publish(msg)
                self.pub_image.publish(msg)

            # self.hex_pub.publish(f"{hex_color} {color_name}")

        except CvBridgeError as e:
            rospy.logerr("CVBridge error: %s", e)
        except Exception as e:
            rospy.logerr("Unexpected error in image_cb: %s", e)

    def get_dominant_color(self, image):
        """Get the most dominant BGR color in the image using KMeans."""
        img = image.reshape((-1, 3)).astype(np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        _, _, centers = cv2.kmeans(img, 1, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        bgr = tuple(map(int, centers[0]))
        r, g, b = bgr[2], bgr[1], bgr[0]
        hex_color = '#{:02x}{:02x}{:02x}'.format(r, g, b)
        return hex_color, bgr #(bgr[0], bgr[1], bgr[2])  # Return BGR

    def decide_movement(self,closest_name):
        if closest_name.lower() == "sienna":
            # self.firstStraight = False
            # rospy.loginfo("Matched Sienna! Moving forward.")
            # self.motion.move_forward(0.08, speed=0.1)
            return "GO"
        elif closest_name.lower() == "dimgray":
            return "GO"
        elif closest_name.lower() == "black":
            return "STOP"
            # rospy.loginfo("Matched black! Turn 20 degrees.")
            # self.motion.turn(20, speed=0.1)
            # self.firstStraight = True
        elif "teal" in closest_name.lower():
            # rospy.loginfo("Matched Albert Heijn Blue!")
            return "STOP"
        else:
            return False
            # rospy.loginfo("Matched Albert Heijn Blue! Turn -20 degrees.")
            # self.motion.turn(-20, speed=0.1)
            # self.firstStraight = True

    def cleanup(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        rospy.init_node("processing_node_colour", xmlrpc_port=45400, tcpros_port=45401)
        CameraProcessingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass