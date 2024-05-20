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
mp_drawing = mp.solutions.drawing_utils
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
class SimpleCNN(nn.Module):
    def __init__(self):
        super(SimpleCNN, self).__init__()
        self.conv1 = nn.Conv2d(1, 32, 3, padding=1)  
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
model.load_state_dict(torch.load(os.path.join(rospkg.RosPack().get_path('final_project'), 'simon_model_v1.pth'), map_location=torch.device("cpu")))
model.eval()

def get_grayscale(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, (9, 9), 0)

    contrast_enhanced = cv2.equalizeHist(blurred_image)

    target_size = 128
    height, width = contrast_enhanced.shape
    scale = target_size / max(height, width)
    resized_image = cv2.resize(contrast_enhanced, (int(width * scale), int(height * scale)), interpolation=cv2.INTER_AREA)

    delta_w = target_size - resized_image.shape[1]
    delta_h = target_size - resized_image.shape[0]
    top, bottom = delta_h // 2, delta_h - (delta_h // 2)
    left, right = delta_w // 2, delta_w - (delta_w // 2)

    final_image = cv2.copyMakeBorder(resized_image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=[0, 0, 0])

   # plt.figure(figsize=(3, 3))
   # plt.imshow(final_image, cmap='gray')
   # plt.title("Processed Image")
   # plt.axis('off')
   # plt.show()

    return final_image

# Define image preprocessing function
def preprocess_image(image, hand_landmarks):
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

    return get_grayscale(cropped_image)


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

# Set white balance
color_sensor = pipeline.get_active_profile().get_device().first_color_sensor()
color_sensor.set_option(rs.option.white_balance, 4600)

align_to = rs.stream.color
align = rs.align(align_to)

class HandTracker:
    def __init__(self):
        rospy.init_node('hand_tracker', anonymous=True)
        self.hand_pub = rospy.Publisher("hand_center", PointStamped, queue_size=10)
        self.hand_state_pub = rospy.Publisher("hand_open", Bool, queue_size=10)
        self.pixels_per_meter = None  # To store the conversion factor
        self.distance_from_floor = None
    
    def publish_hand_center(self, x, y, z):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "camera_color_optical_frame"
        point.point = Point(x, y, z)
        self.hand_pub.publish(point)

    def detect_ar_tag_and_calibrate(self, image, ar_tag_size=0.165):
        """
        Detects an AR tag and calibrates the pixels per meter conversion factor.
        :param image: Input image in which to detect the AR tag.
        :param ar_tag_size: The real-world size of the AR tag in meters (default is 16.5 cm).
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
        self.hand_state_pub.publish(is_open)

    def run(self):
        transform = transforms.Compose([
            transforms.Resize((128, 128)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5], std=[0.5])
        ])
        try:
            cv2.namedWindow("Hand Tracking", cv2.WINDOW_AUTOSIZE)  # OpenCV window
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
                       
                        # Draw the hand landmarks
                        mp_draw.draw_landmarks(color_image_rgb, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                        # Calculate bounding box
                        bbox = self.calculate_hand_bbox(hand_landmarks)
                        x, y, w, h = bbox
                        x_center = x + w / 2
                        y_center = y + h / 2

                        # convert to center
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
                            print("Hand detected, processing gesture")


                            
                            hand_landmarks = results.multi_hand_landmarks[0]
                            color_image_rgb_2 = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                            hand_image = preprocess_image(color_image_rgb_2, hand_landmarks)
                            hand_image_pil = Image.fromarray(hand_image)
                            
                            hand_image_tensor = transform(hand_image_pil).unsqueeze(0).to("cpu")
                            print(DEVICE)
                  
                            with torch.no_grad():
                                output = model(hand_image_tensor)
                                predicted = torch.argmax(output, dim=1).item()
                                is_open = predicted == 1

                        # Display hand state
                        self.publish_hand_state(is_open)
                        cv2.putText(color_image_rgb, "Hand: {}, Coords (m): {:.4f},{:.4f},{:.4f}".format("Open" if is_open else "Closed",x_center_final, y_center_final, hand_dist_from_floor), (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                      #  cv2.putText(color_image_rgb, "State: {}".format("Open" if is_open else "Closed"), (10, 30),
                                 #   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

                # Convert back to BGR for display
                color_image_bgr = cv2.cvtColor(color_image_rgb, cv2.COLOR_RGB2BGR)
                cv2.imshow("Hand Tracking", color_image_bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            pipeline.stop()
            cv2.destroyAllWindows()

    def calculate_hand_bbox(self, hand_landmarks):
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
