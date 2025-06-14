#!/usr/bin/env python2

import cv2
import rospy
from sensor_msgs.msg import CompressedImage

# Constants
FORMAT = "jpeg"
COMPRESSED_EXT = ".jpg"

class CameraNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.loginfo("Initializing camera publisher node...")
        rospy.init_node("publisher_node", anonymous=True)

        # Publisher for the image topic
        self.image_pub = rospy.Publisher(
            '/camera/image_raw', 
            CompressedImage,
            queue_size=1)
        
        # GStreamer pipeline for the Jetson CSI camera
        self.sensor_id = rospy.get_param("~sensor_id")
        self.width = rospy.get_param("~width")
        self.height = rospy.get_param("~height")
        self.fps = rospy.get_param("~fps")
        self.pipeline = self.gstreamer_pipeline()
        if self.pipeline is None:
            rospy.logerr("Pipeline could not be initialized!")

        # OpenCV video capture with the GStreamer pipeline
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

        # Check camera
        if not self.cap.isOpened():
            rospy.logerr("Unable to open camera")
        
        rospy.loginfo("Camera publisher node initialized!!!")
        self.start_publishing(self.fps, FORMAT)

    def start_publishing(self, fps, format):
        rate = rospy.Rate(fps)
        msg = CompressedImage()
        msg.format = format

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Failed to capture image")
                continue
            try:
                # Convert the OpenCV image to a ROS CompressedImage message
                success, encoded_image = cv2.imencode(COMPRESSED_EXT, frame)
                if success:
                    msg.header.stamp = rospy.Time.now()
                    msg.data = encoded_image.tobytes()
                    self.image_pub.publish(msg)

            except Exception as e:
                rospy.logerr("Error converting image: {}".format(e))
            # Sleep to maintain the publishing rate
            rate.sleep()
            
    def gstreamer_pipeline(self):
        if self.sensor_id == 0:
            return ('nvarguscamerasrc sensor-id=0 ! '
                    'video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, '
                    'format=(string)NV12, framerate=(fraction)%d/1 ! '
                    'queue max-size-buffers=1 leaky=downstream ! '
                    'nvvidconv ! video/x-raw, format=(string)I420 ! '
                    'videoconvert ! video/x-raw, format=(string)BGR ! '
                    'queue max-size-buffers=1 leaky=downstream ! '
                    'appsink drop=true sync=false' % (self.width, self.height, self.fps))

        elif self.sensor_id == 1:
            return ('v4l2src device=/dev/video1 ! '
                    'video/x-raw, width=(int)%d, height=(int)%d, '
                    'format=(string)YUY2, framerate=(fraction)%d/1 ! '
                    'videoconvert ! video/x-raw, format=(string)BGR ! '
                    'appsink' % (self.width, self.height, self.fps))
        else:
            return None

if __name__ == "__main__":
    camera_pub = None
    try:
        camera_pub = CameraNode()
    except rospy.ROSInterruptException:
        if camera_pub is not None:
            camera_pub.cap.release()
