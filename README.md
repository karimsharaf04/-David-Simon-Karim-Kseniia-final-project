# Intro Robotics Final Project
### Group members: David Suh, Simon Mahns, Karim Sharaf, Kseniia Korotenko

### Project Description:
We are using a depth camera and hand gesture recognition to teach the turtlebots to play a game that involves
dragging and dropping tubes according to commands that human players execute via hand gestures. The robot that
gains more points by knocking over tubes (+1 point) and dragging and dropping tubes in the goal area (+2 points)
in a limited time wins. \

This project is exciting because we combined two areas of interest: \
— hand gesture recognition (computer vision) using ML \
— robot arm manipulation \
The result is an HCI/HRI project that augments the capabilities of the robot.

### System Architecture:
Describe in detail the robotics algorithm you implemented and each major component of your project, highlight what pieces of code contribute to these main components \
**Computer Vision Component: Hand Gesture Detection** \
This code is in the file `hand_detection.py`. The rospy node is called `hand_tracker`. First, in this file, load the CNN model. We also do some logistics such as initializing MediaPipe and the RealSense depth camera. We also set the white balance to make sure the image is processed correctly. When the `hand_tracker` node runs, it gets an image from the realsense camera, draws hand landmarks, calculates bounding box for the hand, crops the image, and preprocesses the resulting image of a hand to be fed to the model. (The preprocessing imolves some resizing, format change, normalization etc.) Then we get the model's prediction, and publish the hand position and state (open/closed) as based on this prediction. Then we display the image for debugging. \
**Inverse Kinematics Component: Robot Arm Manipulation** \
This code is in the file `arm_motion.py`. The ros node running in this file is called `arm_control`. It initializes arm, resets arm and gripper position, and sets up subscribers on initialization. The heart of this node are two callback functions. The first one is `hand_position_callback` that moves the robot arm according to the hand position received (`PointStamped` type). The frequency of update is capped to avoid bugs. We get current arm joint values, and use helper functions `_get_joint_1`, `_get_joint_2`, `_get_joint_3` to compute the desired joint positions based on the received ros message. Then we move the arm and stop after the movement is complete. The second callback function, `hand_state_callback`, updates the gripper position to be open or closed based on the Boolean message received. \
As for the helper functions: \
`_get_joint_1` gets the position for the swivel servo, mapping the `x, y` hand position to a $0$ - $180$ degree range ($0$ - $\pi$ radians). \
`_get_joint_2` gets the position for the tilt servo, mapping the distance of the hand from the ground in meters to a $0$ - $90$ degree range ($0$ - $\frac{\pi}{2}$ radians). \
`_get_joint_3` gets the position for the elbow join, mapping the angular distance of the hand from the origin to a $0$ - $90$ degree range ($0$ - $\frac{\pi}{2}$ radians). \

### ROS Node Diagram:
Please include a visual diagram representing all of the ROS nodes, ROS topics, and publisher/subscriber connections present in your final project. \
We have two nodes: `hand_tracker` in `hand_detection.py` and `arm_control` in `arm_motion.py`. `hand_tracker` publishes to ros topics `hand_center` (hand position, PointStamped message) and `hand open`(flag which determines whether the hand should be open, Bool message). The node `arm_control` is subscribed to these topics. \
![Ros Node Diagram](<Screenshot 2024-05-23 at 1.09.33 PM-1.png>)

### Execution:
Describe how to run your code, e.g., step-by-step instructions on what commands to run in each terminal window to execute your project code. \
We created a launch file `final.launch`. So it is enough to run: `roslaunch final_project final.launch`. This launch file launches OpenMANIPULATOR Bringup, MoveIt configuration for the robot arm, and runs the hand detection and arm motion nodes. (The data stream from the depth camera is set up in the hand detection node.) \
The second step is for a human to position themselves under the depth camera and operate the robot using hand gestures! If we want two (or more) robots to compete, we need two (or more) identical setups.\

### Challenges, Future Work, and Takeaways:

**Challenges:** \
Perhaps the biggest challenge was learning to work with the MoveIt framework. While we started out with the inverse kinematics theory from class slides, we quickly realized that the actual limitations of the robot arm, and what is given to us in the MoveIt API, mean that we have to adjust our plans. We ended up implementing a simpler model than we originally planned, making sure that each joint corresponds to one specific direction of movement. \
It is also interesting that our ML model for hand gesture detention is homebrew! Adjusting the parameters to make this model work was another challenge.

**Future Work:** \
— No moveit. Servo controiller -> raw programming
— Do on the pi directly

**Takeaways:** \
— Learning how to use a depth camera and what we can accomplish with it! \
— learning how to
— Inverse kinematics

