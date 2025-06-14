#!/usr/bin/env python3
"""
Camera Processing Node for Color Detection and Movement Decision

This ROS node subscribes to a camera image stream, detects the dominant color in each frame,
determines the closest CSS3 color name, and publishes a movement command based on the color.

The node terminates itself upon detecting any color containing "green" in its name.
"""
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
import webcolors
from scipy.spatial import KDTree

class ColorNamer:
    """
    Utility class to convert RGB values to the closest named CSS3 color
    using a KDTree for fast nearest-neighbor search.
    """
    def __init__(self):
        self.names = webcolors.names("css3")  # List of color names
        self.rgb_values = []

        for name in self.names:
            hex_code = webcolors.name_to_hex(name)
            self.rgb_values.append(webcolors.hex_to_rgb(hex_code))

        # Store values in a k-d tree for efficience
        self.kdtree = KDTree(self.rgb_values)

    def name_from_rgb(self, rgb_tuple):
        """
        Get the closest named CSS3 color to a given RGB triplet.
        """
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

        # Subscribe to the undistorted camera image stream
        self.sub_image = rospy.Subscriber(
            "/camera/image_undistorted",
            CompressedImage,
            self.image_cb,
            queue_size=1,
            buff_size=2**24
        )

        # self.processed_image_pub = rospy.Publisher("/camera/processed_image_colour", CompressedImage, queue_size=1)
        # self.hex_pub = rospy.Publisher("/camera/dominant_color_hex", String, queue_size=1)
        
        # Publisher for processed annotated image (We should remove annotation if we want clear feed)
        self.pub_image = rospy.Publisher(
            "/camera/processed_image_colour",
            CompressedImage,
            queue_size=1
        )

        # Publisher for color-based motion command
        self.colour_pub = rospy.Publisher(
            "/motion/colour_motion_command",
            String,
            queue_size=1)


        self.first_image_received = False
        rospy.loginfo("Camera processing node initialized.")

    def image_cb(self, data):
        """
        Callback function for processing each image.
        Determines dominant color and publishes movement command based on it.
        """
        if not self.first_image_received:
            self.first_image_received = True
            self.terminate = False
            rospy.loginfo("Camera subscriber captured first image.")

        try:
            # Decode and resize image
            cv_image = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)

            hex_color, bgr = self.get_dominant_color(cv_image)      # get dominant colour
            rgb = (bgr[2], bgr[1], bgr[0])                          # Rearrange RGB from BGR
            color_name = self.color_namer.name_from_rgb(rgb)        # Get name from the RGB triplet

            
            label = f"{hex_color} {color_name}"

            move = self.decide_movement(color_name)
            
            if "green" in color_name.lower():
                rospy.loginfo("GREEN DETECTED!")
                # self.terminate = True                       # If you see a flag with a colour that contains "Green" you will start moving

            if move:
                self.colour_pub.publish(move)
            else:
                pass

            if self.terminate:                                                          # Terminate node since you only start with this node.
                rospy.loginfo("Termination condition met. Shutting down node.")
                rospy.signal_shutdown("Green color detected")
                return

            # Annotate image
            cv2.rectangle(cv_image, (0, 0), (250, 50), bgr, -1)     # It doesn't work anymore
            cv2.putText(cv_image, label, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)

            success, encoded_image = cv2.imencode(".jpg", cv_image)
            if success:
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                msg.data = encoded_image.tobytes()
                # self.processed_image_pub.publish(msg)                         # Old processing
                self.pub_image.publish(msg)

            # self.hex_pub.publish(f"{hex_color} {color_name}")                 # Old publisher

        except CvBridgeError as e:
            rospy.logerr("CVBridge error: %s", e)
        except Exception as e:
            rospy.logerr("Unexpected error in image_cb: %s", e)

    def get_dominant_color(self, image):
        """
        Uses KMeans clustering to find the dominant color in the image.

        Returns:
            hex_color: Hexadecimal color string (e.g. '#ff0000')
            bgr: Dominant color in BGR tuple
        """
        img = image.reshape((-1, 3)).astype(np.float32)

        # Define criteria and apply KMeans clustering with 1 cluster
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        _, _, centers = cv2.kmeans(img, 1, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        bgr = tuple(map(int, centers[0]))

        # Convert BGR to RGB and then to hex (why is it like this?)
        r, g, b = bgr[2], bgr[1], bgr[0]
        hex_color = '#{:02x}{:02x}{:02x}'.format(r, g, b)
        return hex_color, bgr #(bgr[0], bgr[1], bgr[2])  # Return BGR

    def decide_movement(self,closest_name):
        """
        Decide movement command string based on the closest color name.

        Returns:
            A string movement command, or False if no command is required.
        """
        if closest_name.lower() == "sienna":
            # self.firstStraight = False
            # rospy.loginfo("Matched Sienna! Moving forward.")
            # self.motion.move_forward(0.08, speed=0.1)
            return "STOP"
        # elif closest_name.lower() == "dimgray":
        #     return "GO"
        # elif closest_name.lower() == "black":
        #     return "STOP"
        #     # rospy.loginfo("Matched black! Turn 20 degrees.")
        #     # self.motion.turn(20, speed=0.1)
        #     # self.firstStraight = True
        elif "green" in closest_name.lower():
            # rospy.loginfo("Matched Albert Heijn Blue!")
            return "GO 0.02"
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