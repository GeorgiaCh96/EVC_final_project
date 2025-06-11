#!/usr/bin/env python3
import cv2, rospy, numpy
from sensor_msgs.msg import CompressedImage


class ViewNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.loginfo("Initializing view node........")
        rospy.init_node('view_node')

        # Construct subscriber
        self.sub_barcode = rospy.Subscriber(
            "/camera/processed_image_barcode",
            CompressedImage,
            self.show_barcode,
            buff_size=2**24,
            queue_size=None
        )

        self.sub_gestures = rospy.Subscriber(
            "/camera/processed_image_gestures",
            CompressedImage,
            self.show_gestures,
            buff_size=2**24,
            queue_size=None
        )

        self.sub_gestures = rospy.Subscriber(
            "/camera/processed_image_motion",
            CompressedImage,
            self.show_motion,
            buff_size=2**24,
            queue_size=None
        )

        self.sub_gestures = rospy.Subscriber(
            "/camera/yolo_processed_image",
            CompressedImage,
            self.show_motion,
            buff_size=2**24,
            queue_size=None
        )


        rospy.loginfo("Camera subscriber node initialized!")

    def show_barcode(self, data):
        self.display_image(data, window_name="Barcode View")

    def show_gestures(self, data):
        self.display_image(data, window_name="Gesture View")

    def show_motion(self, data):
        self.display_image(data, window_name="Gesture View")

    def display_image(self, data, window_name):
        try:
            img = cv2.imdecode(numpy.frombuffer(data.data, numpy.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                cv2.imshow(window_name, img)
                cv2.waitKey(1)
        except Exception as err:
            rospy.logerr(f"Error displaying {window_name}: {err}")

if __name__ == "__main__":
    view_node = None
    try:
        view_node = ViewNode()
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
