#!/usr/bin/env python3
import cv2, rospy, numpy
from sensor_msgs.msg import CompressedImage


class ViewNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.loginfo("Initializing view node...")
        rospy.init_node('view_node', anonymous=True)

        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/processed_image",
            CompressedImage,
            self.show,
            buff_size=2**24,
            queue_size=None
        )

        rospy.loginfo("Camera subscriber node initialized!")

    def show(self, data):
        try:
            # Decode image without CvBridge
            img = cv2.imdecode(numpy.frombuffer(data.data, numpy.uint8), cv2.IMREAD_COLOR)

            # Ensure the window updates instantly
            cv2.imshow("Webcam View", img)
            cv2.waitKey(1)
        except Exception as err:
            rospy.logerr("Error converting image: {}".format(err))

if __name__ == "__main__":
    view_node = None
    try:
        view_node = ViewNode()
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
