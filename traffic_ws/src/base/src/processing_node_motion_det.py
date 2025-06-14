#!/usr/bin/env python3
import rospy, numpy, cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# Constants
FILENAME = "motion.avi"
DETECTION_THRESHOLD = 1000

class MotionNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.loginfo("Initializing processing node motion det...")
        rospy.init_node("processing_node_motion_det", anonymous=True, xmlrpc_port=45300, tcpros_port=45301)

        # Set time buffer
        self.count = 0

        self.locked = False

        # Configure cv2
        self.fgbg = cv2.createBackgroundSubtractorMOG2()
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        self.width = 640
        self.height = 480
        self.out = cv2.VideoWriter(FILENAME,
                                   fourcc,
                                   15,
                                   (self.width * 2, self.height))
        
        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_undistorted",
            CompressedImage,
            self.track,
            buff_size=2**24,
            queue_size=None
        )
        
        # Construct area publisher
        self.pub_area = rospy.Publisher(
            "/motion/nomotion_motion_command",
            String,
            queue_size=1,
        )

        rospy.loginfo("Processing node motion det initialized!!!")

    def track(self, msg):
        try:
            # Decode image
            tracked_img = cv2.imdecode(numpy.frombuffer(msg.data, numpy.uint8), cv2.IMREAD_COLOR)

            # Apply background subtraction
            fgmask = self.fgbg.apply(tracked_img)
            
            # Apply thresholding to remove noise
            th = cv2.threshold(fgmask, 200, 255, cv2.THRESH_BINARY)[1]
            
            # Find contours
            contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Processing variables
            max_area = 0
            max_contour = None
            
            # Find center
            for contour in contours:
                # Add to center totals
                x, y, w, h = cv2.boundingRect(max_contour)
                # Draw bounding box around object
                cv2.rectangle(tracked_img, (x, y), (x+w, y+h), (255, 0, 0), 2)
                # Find max
                area = cv2.contourArea(contour)
                if  area > max_area:
                    max_area = area
                    max_contour = contour

            report = String()
            
            # Show on image
            if (max_area > DETECTION_THRESHOLD):
                # Report motion
                if not self.locked: 
                    report.data = "LOCK"
                    self.pub_area.publish(report)
                    self.locked = True
                # Get bounding box coordinates
                x, y, w, h = cv2.boundingRect(max_contour)
                # Draw bounding box around object
                cv2.rectangle(tracked_img, (x, y), (x+w, y+h), (0, 0, 255), 2)
                 # Add text
                cv2.putText(tracked_img, f"Movement Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            else:
                # Report motion
                if self.locked:
                    report.data = "UNLOCK"
                    self.pub_area.publish(report)
                    self.locked = False
                # Add text
                cv2.putText(tracked_img, 'No Movement Detected', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

            # Show frame
            cv2.imshow('Tracked', tracked_img)
            cv2.waitKey(1)
            
            # Save frame to video
            self.out.write(tracked_img)
        except Exception as e:
            rospy.logerr("Failed to track motions: {}".format(e))

if __name__ == "__main__":
    motion_node = None
    try:
        motion_node = MotionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        motion_node.out.release()
        cv2.destroyAllWindows()
