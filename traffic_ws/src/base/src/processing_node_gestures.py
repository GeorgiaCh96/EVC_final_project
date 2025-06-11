#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import mediapipe as mp


class CameraProcessingNode:
    def __init__(self):
        rospy.loginfo("Initializing camera processing node...")
        rospy.init_node('processing_node_gestures', xmlrpc_port=45200, tcpros_port=45201)

        self.bridge = CvBridge()

        # MediaPipe hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False,
                                         max_num_hands=1,
                                         min_detection_confidence=0.7,
                                         min_tracking_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils

        # ROS I/O
        self.sub_image = rospy.Subscriber(
            "/camera/image_undistorted", CompressedImage,
            self.image_cb, buff_size=2**24, queue_size=1
        )
        self.annotated_pub = rospy.Publisher("/camera/processed_image_gestures", CompressedImage, queue_size=1)
        self.gesture_pub = rospy.Publisher("/motion/gesture_motion_command", String, queue_size=1)

        self.first_image_received = False
        rospy.loginfo("Camera processing node initialized!")

    def image_cb(self, data):
        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("Camera subscriber captured first image.")

        try:
            cv_image = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)
            img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            results = self.hands.process(img_rgb)

            annotated_image = cv_image.copy()
            gesture_text = "No gesture"

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_draw.draw_landmarks(
                        annotated_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                    # Detect gesture
                    gesture = self.detect_gesture(hand_landmarks)
                    if gesture:
                        gesture_text = gesture
                        self.gesture_pub.publish(gesture)
                        rospy.loginfo(f"{gesture} gesture detected")
            #         else:
            #             #rospy.loginfo("No recognizable gesture")
            # else:
            #     rospy.loginfo("No hand detected")

            # Annotate image with gesture name
            cv2.putText(annotated_image, f"Gesture: {gesture_text}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            # Compress and publish as CompressedImage
            success, encoded_image = cv2.imencode(".jpg", annotated_image)
            if success:
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                msg.data = encoded_image.tobytes()
                self.annotated_pub.publish(msg)

        except CvBridgeError as err:
            rospy.logerr("CvBridge error: {}".format(err))
        except Exception as e:
            rospy.logerr("Unhandled exception: {}".format(e))

    def detect_gesture(self, hand_landmarks):
        """
        Detects:
        - STOP (All fingers up)
        - GO FORWARD (Peace sign: index and middle up)
        """
        landmarks = hand_landmarks.landmark

        def is_up(tip_idx, pip_idx):
            return landmarks[tip_idx].y < landmarks[pip_idx].y

        # Finger up status
        thumb = is_up(4, 2)
        index = is_up(8, 6)
        middle = is_up(12, 10)
        ring = is_up(16, 14)
        pinky = is_up(20, 18)

        if all([thumb, index, middle, ring, pinky]):
            "GO 100 0.3"
        # elif index and middle and not (thumb or ring or pinky):
        #     return "GO 100 0.3"
        else:
            return None

    def cleanup(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        camera_node = CameraProcessingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        camera_node.cleanup()
