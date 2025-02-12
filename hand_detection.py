#!/usr/bin/env python3

import matplotlib.pyplot as plt
import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
import mediapipe as mp
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, Point
import torch
from torchvision import transforms
from PIL import Image
import torch.nn as nn
import torch.optim as optim
import os
import rospkg

FRAME_RATE = 30 # Set the frame rate
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu") # Use GPU if available
mp_drawing = mp.solutions.drawing_utils 


class SimpleCNN(nn.Module):
    """
    Simple CNN model for hand gesture classification
    """
    def __init__(self):
        super(SimpleCNN, self).__init__()
        self.conv1 = nn.Conv2d(1, 32, 3, padding=1)  # 1 input channel, 32 output channels, 3x3 kernel
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(32, 64, 3, padding=1)
        self.fc1 = nn.Linear(64 * 32 * 32, 256) # 32x32 is the size of the image after 2 pooling layers
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
model.load_state_dict(torch.load(os.path.join(rospkg.RosPack().get_path('final_project'), 'final_model.pth'), map_location=torch.device("cpu")))
model.eval()


# Init MediaPipe for detecting hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_draw = mp.solutions.drawing_utils

# Init RealSense pipeline (depth and color)
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, FRAME_RATE)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, FRAME_RATE)
pipeline.start(config)

# Set white balance
color_sensor = pipeline.get_active_profile().get_device().first_color_sensor()
color_sensor.set_option(rs.option.white_balance, 4600)

# Align depth and color frames
align_to = rs.stream.color
align = rs.align(align_to)

class HandTracker:
    """
    HandTracker class to detect hand landmarks and track the hand center in 3D space.
    """
    def __init__(self):
        rospy.init_node('hand_tracker', anonymous=True)
        self.hand_pub = rospy.Publisher("hand_center", PointStamped, queue_size=10)
        self.hand_state_pub = rospy.Publisher("hand_open", Bool, queue_size=10)
        self.pixels_per_meter = None  # To store the conversion factor
        self.distance_from_floor = None
    
    def publish_hand_center(self, x, y, z):
        """
        Publishes the 3D coordinates of the hand center for consuming by the hand_control node.
        """
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "camera_color_optical_frame"
        point.point = Point(x, y, z)
        self.hand_pub.publish(point)

    def detect_ar_tag_and_calibrate(self, image, ar_tag_size=0.165):
        """
        Detects an AR tag and calibrates the pixels per meter conversion factor.
        """ 
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)



        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, _, _ = detector.detectMarkers(gray)

        if corners:
            top_left, top_right, _, _ = corners[0][0]
            tag_width_pixels = np.linalg.norm(top_right - top_left)
            self.pixels_per_meter = tag_width_pixels / ar_tag_size
            print(f"Calibrated Pixels per Meter: {self.pixels_per_meter}")
            
    def publish_hand_state(self, is_open):
        """
        Publishes the state of the hand (open or closed) for consuming by the arm_control node.
        """
        self.hand_state_pub.publish(is_open)

    def run(self):
        """
        Main loop to detect hands, track hand center, and classify hand state.
        """
        transform = transforms.Compose([
            transforms.Resize((128, 128)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5], std=[0.5])
        ])
        try:
            cv2.namedWindow("Hand Tracking", cv2.WINDOW_AUTOSIZE) # creates the window for debugging
            while not rospy.is_shutdown():

                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                if not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                color_image_rgb_raw = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                color_image_rgb = color_image_rgb_raw

                if self.pixels_per_meter is None:
                    self.detect_ar_tag_and_calibrate(color_image_rgb_raw)

                if self.distance_from_floor is None:
                    self.distance_from_floor = self.calculate_hand_depth(640/2, 480/2, aligned_frames.get_depth_frame())
                    print("Distance from floor: ", self.distance_from_floor)
                results = hands.process(color_image_rgb)
                if results.multi_hand_landmarks:

                    for hand_landmarks in results.multi_hand_landmarks:
                       

                        # Calculate bounding box
                        bbox = self.calculate_hand_bbox(hand_landmarks)
                        x, y, w, h = bbox
                        x_center = x + w / 2
                        y_center = y + h / 2

                        # get the center
                        new_x = x_center- 640/2
                        new_y = 480/2 - y_center

                        # convert pixel to meter
                        x_center_final = new_x / self.pixels_per_meter
                        y_center_final = new_y / self.pixels_per_meter

                        dist_from_cam = self.calculate_hand_depth(x_center, y_center, aligned_frames.get_depth_frame())
                        hand_dist_from_floor = self.distance_from_floor - dist_from_cam
                        self.publish_hand_center(x_center_final, y_center_final, hand_dist_from_floor)
                        
                        # Draw bounding box around the detected hand
                        cv2.rectangle(color_image_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)

                        # Crop and preprocess image for model input
                        is_open = False
                        if results.multi_hand_landmarks:
                            hand_landmarks = results.multi_hand_landmarks[0]
                            color_image_rgb_2 = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                            hand_image = self.preprocess_image(color_image_rgb_2, hand_landmarks)
                            
                            # prepare the image for the model
                            hand_image_pil = Image.fromarray(hand_image)
                            hand_image_tensor = transform(hand_image_pil).unsqueeze(0).to("cpu")
                            with torch.no_grad():
                                output = model(hand_image_tensor)
                                predicted = torch.argmax(output, dim=1).item()
                                is_open = predicted == 1

                        # publish the hand state
                        self.publish_hand_state(is_open)

                        # Draw all states of the hand on the cv2 window
                        cv2.putText(color_image_rgb, "Hand: {}, Coords (m): {:.4f},{:.4f},{:.4f}".format("Open" if is_open else "Closed",x_center_final, y_center_final, hand_dist_from_floor), (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
 
                # Convert the back to BGR for display
                color_image_bgr = cv2.cvtColor(color_image_rgb, cv2.COLOR_RGB2BGR)
                cv2.imshow("Hand Tracking", color_image_bgr)

                # for early exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            pipeline.stop()
            cv2.destroyAllWindows()

    def calculate_hand_bbox(self, hand_landmarks):
        """
        Helper function that calculates the bounding box of the hand landmarks
        """
        if not hand_landmarks:
            return 0, 0, 0, 0
        x_coords = [lm.x for lm in hand_landmarks.landmark]
        y_coords = [lm.y for lm in hand_landmarks.landmark]
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        x_min, x_max = int(x_min * 640), int(x_max * 640)
        y_min, y_max = int(y_min * 480), int(y_max * 480)
        return x_min, y_min, x_max - x_min, y_max - y_min

    def calculate_hand_depth(self, x, y, depth_frame):
        """
        Helper function to calculate the depth of the hand at a given x,y coordinate
        """
        if not depth_frame:
            return 0  #if the depth frame is None
        
        # ensure x and y are within the valid range, normalize the values
        x = max(0, min(int(x), depth_frame.get_width() - 1))
        y = max(0, min(int(y), depth_frame.get_height() - 1))
        return depth_frame.get_distance(x, y)


    def get_grayscale(self, image):
        """
        Helper function to preprocess hand image by converting it to grayscale, blurring, and resizing.
        This makes the image suitable for input to the model and increases the accuracy of the model.
        """

        # greyscale, blur, and increase contrast
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred_image = cv2.GaussianBlur(gray_image, (9, 9), 0)
        contrast_enhanced = cv2.equalizeHist(blurred_image)

        # resize to 128x128, keeping hand centered
        target_size = 128
        height, width = contrast_enhanced.shape
        scale = target_size / max(height, width)
        resized_image = cv2.resize(contrast_enhanced, (int(width * scale), int(height * scale)), interpolation=cv2.INTER_AREA)

        delta_w = target_size - resized_image.shape[1]
        delta_h = target_size - resized_image.shape[0]
        top, bottom = delta_h // 2, delta_h - (delta_h // 2)
        left, right = delta_w // 2, delta_w - (delta_w // 2)

        final_image = cv2.copyMakeBorder(resized_image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=[0, 0, 0])

        # for testing
        # plt.figure(figsize=(3, 3))
        # plt.imshow(final_image, cmap='gray')
        # plt.title("Processed Image")
        # plt.axis('off')
        # plt.show()

        return final_image

    def preprocess_image(self, image, hand_landmarks):
        """
        Preprocesses the image by cropping the hand region and then converting it to grayscale (and a few other things).
        This cropping is necessary for getting the model to work well. It maintains the 1:1 aspect ratio as well.
        """
        h, w, _ = image.shape
        x_min, x_max = int(min(lm.x for lm in hand_landmarks.landmark) * w), int(max(lm.x for lm in hand_landmarks.landmark) * w)
        y_min, y_max = int(min(lm.y for lm in hand_landmarks.landmark) * h), int(max(lm.y for lm in hand_landmarks.landmark) * h)

        # Adjust to keep the aspect ratio 1:1 and include padding
        side_length = max(x_max - x_min, y_max - y_min)
        padding = int(side_length * 0.1)
        x_min = max(0, x_min - padding)
        x_max = min(w, x_max + padding)
        y_min = max(0, y_min - padding)
        y_max = min(h, y_max + padding)

        cropped_image = image[y_min:y_max, x_min:x_max]

        return self.get_grayscale(cropped_image)

if __name__ == "__main__":
    ht = HandTracker()
    ht.run()
