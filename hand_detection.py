#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
import mediapipe as mp
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped, Point
import torch
from torchvision import transforms
from PIL import Image
import torch.nn as nn
import torch.optim as optim

# Define the CNN model architecture
class SimpleCNN(nn.Module):
    def __init__(self):
        super(SimpleCNN, self).__init__()
        self.conv1 = nn.Conv2d(3, 32, 3, padding=1)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(32, 64, 3, padding=1)
        self.fc1 = nn.Linear(64 * 32 * 32, 256)
        self.fc2 = nn.Linear(256, 2)

    def forward(self, x):
        x = self.pool(torch.relu(self.conv1(x)))
        x = self.pool(torch.relu(self.conv2(x)))
        x = torch.flatten(x, 1)
        x = torch.relu(self.fc1(x))
        x = self.fc2(x)
        return x

# Load the model
model = SimpleCNN()
model.load_state_dict(torch.load('/home/ksharaf/catkin_ws/src/-David-Simon-Karim-Kseniia-final-project/model_closed_hand.pth'))
model.eval()

# Define image preprocessing function
def preprocess_image(image):
    transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize((128, 128)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    return transform(image).unsqueeze(0)  # Add batch dimension

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
        self.hand_pub = rospy.Publisher("hand_center", PointStamped, queue_size=10)
        self.hand_state_pub = rospy.Publisher("hand_open", Bool, queue_size=10)

    def publish_hand_center(self, x, y, z):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "camera_color_optical_frame"
        point.point = Point(x, y, z)
        self.hand_pub.publish(point)

    def publish_hand_state(self, is_open):
        self.hand_state_pub.publish(is_open)

    def run(self):
        try:
            cv2.namedWindow("Hand Tracking", cv2.WINDOW_AUTOSIZE)  # Create an OpenCV window
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
                        mp_draw.draw_landmarks(color_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)  # Draw hand landmarks
                        x, y = self.calculate_hand_center(hand_landmarks)
                        z = self.calculate_hand_depth(x, y, aligned_frames.get_depth_frame())
                        self.publish_hand_center(x, y, z)
                        
                        # Prepare the image for the model
                        hand_image = preprocess_image(color_image_rgb)
                        with torch.no_grad():
                            output = model(hand_image)
                            _, predicted = torch.max(output, 1)
                            is_open = True if predicted.item() == 1 else False
                        self.publish_hand_state(is_open)

                        # Show hand state on the image
                        cv2.putText(color_image, "State: {}".format("Open" if is_open else "Closed"), (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

                cv2.imshow("Hand Tracking", cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))  # Display the image
                if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
                    break

        finally:
            pipeline.stop()
            cv2.destroyAllWindows()  # Close the window



    def calculate_hand_center(self, hand_landmarks):
        x_coords = [landmark.x for landmark in hand_landmarks.landmark]
        y_coords = [landmark.y for landmark in hand_landmarks.landmark]
        x_mean = np.mean(x_coords)
        y_mean = np.mean(y_coords)

        x_center = x_mean * 640
        y_center = y_mean * 480  
        return x_center, y_center

    def calculate_hand_depth(self, x, y, depth_frame):
        if not depth_frame:
            return 0  #if the depth frame is None
        #ensure x and y are within the valid range
        x = max(0, min(int(x), depth_frame.get_width() - 1))
        y = max(0, min(int(y), depth_frame.get_height() - 1))
        depth = depth_frame.get_distance(x, y)
        return depth


if __name__ == "__main__":
    ht = HandTracker()
    ht.run()
