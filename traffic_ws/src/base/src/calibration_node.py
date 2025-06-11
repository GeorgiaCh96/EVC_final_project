#!/usr/bin/env python2
import cv2, rospy, numpy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

# Constants
MTX = numpy.array([
    [374.30090078,   0.        , 314.5970749 ],
    [  0.        , 505.95154976, 197.4745109 ],
    [  0.        ,   0.        ,   1.        ]
    ])
DIST = numpy.array([[-0.29800119, -0.03743905, -0.00332081, -0.00080399,  0.11400397]])
FORMAT = "jpeg"
COMPRESSED_EXT = ".jpg"

class ProcessingNode:
    def __init__(self):
        rospy.loginfo("Initializing processing node...")
        rospy.init_node('calibration_node', anonymous=True)
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw",
            CompressedImage,
            self.process,
            buff_size=2**24,
            queue_size=None
        )
        self.process_pub = rospy.Publisher(
            "/camera/image_undistorted",
            CompressedImage,
            queue_size=1
        )
        rospy.loginfo("Camera subscriber node initialized!")

    def process(self, msg):
        new_msg = CompressedImage()
        new_msg.format = FORMAT
        try:
            # Deserialize
            raw = cv2.imdecode(numpy.frombuffer(msg.data, numpy.uint8), cv2.IMREAD_COLOR)
            
            # Extract properties
            h, w = raw.shape[:2]
            
            # Undistort
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(MTX, DIST, (w,h), 1, (w,h))
            dst = cv2.undistort(raw, MTX, DIST, None, newcameramtx)
            x, y, w, h = roi
            
            # Publish message
            success, encoded_image = cv2.imencode(COMPRESSED_EXT, dst[y:y+h, x:x+w])
            if success:
                new_msg.header.stamp = rospy.Time.now()
                new_msg.data = encoded_image.tobytes()
                self.process_pub.publish(new_msg)
        except CvBridgeError as e:
            rospy.logerr("Error converting image: {}".format(e))

if __name__ == "__main__":
    processing_node = None
    try:
        processing_node = ProcessingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if processing_node is not None:
            cv2.destroyAllWindows()
