#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
import mediapipe as mp
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped, Point

# Init MediaPipe 
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_draw = mp.solutions.drawing_utils

# Init RealSense 
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

class HandTracker:
    def __init__(self):
        rospy.init_node('hand_tracker', anonymous=True)
        #  PointStamped message type https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PointStamped.html 
        # ugh we will need to also publish the open / close
        self.hand_pub = rospy.Publisher("hand_center", PointStamped, queue_size=10) 
        self.hand_state_pub = rospy.Publisher("hand_open", Bool, queue_size=10)  # Boolean publisher for hand state


    def publish_hand_center(self, x, y, z):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "camera_color_optical_frame"
        point.point = Point(x, y, z)
        self.hand_pub.publish(point)

    def run(self):
        try:
            while not rospy.is_shutdown():
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                if not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

                results = hands.process(color_image_rgb)
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        x, y = self.calculate_hand_center(hand_landmarks)
                        z = self.calculate_hand_depth(x, y, aligned_frames.get_depth_frame())
                        self.publish_hand_center(x, y, z)
                        # Get hand state
                        self.publish_hand_state(False) # update False means it closed

        finally:
            pipeline.stop()

    def calculate_hand_center(self, hand_landmarks):
        x_coords = [landmark.x for landmark in hand_landmarks.landmark]
        y_coords = [landmark.y for landmark in hand_landmarks.landmark]
        x_mean = np.mean(x_coords)
        y_mean = np.mean(y_coords)

        x_center = x_mean * 640  # wtf is the heigh
        y_center = y_mean * 480  
        return x_center, y_center

    def calculate_hand_depth(self, x, y, depth_frame):
        depth = depth_frame.get_distance(int(x), int(y))
        return depth

if __name__ == "__main__":
    ht = HandTracker()
    ht.run()
