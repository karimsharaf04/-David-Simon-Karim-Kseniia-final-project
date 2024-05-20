#!/usr/bin/python3

import rospy
import sys
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import moveit_commander

class ArmControl(object):
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("arm_control")

        # Initialize MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)

        # Set subscribers for hand position and state
        rospy.Subscriber("hand_center", PointStamped, self.hand_position_callback)
        rospy.Subscriber("hand_open", Bool, self.hand_state_callback)

        # Interface to the group of joints making up the TurtleBot3 open manipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # Interface to the group of joints making up the TurtleBot3 open manipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position to a normal position
        arm_joint_positions = [0, 0, 0, 0]
        self.move_group_arm.go(arm_joint_positions, wait=True)
        self.move_group_arm.stop()

        rospy.sleep(3)  # Give the arm time to perform the action before moving the gripper

        # Reset gripper position
        gripper_joint_open = [0, 0]
        self.move_group_gripper.go(gripper_joint_open, wait=True)
        self.move_group_gripper.stop()

        rospy.sleep(3)

        # Initialize the gripper state
        self.gripper_open = True

    def hand_position_callback(self, data):
        """Callback function for controlling arm based on hand position"""
        x, y, z = data.point.x, data.point.y, data.point.z

        # Calculate angles and distances
        q1 = 0  # Base swivel remains fixed

        # Base tilt (Joint 2)
        if z <= 0:
            q2 = -1.55  # Tilt all the way down
        elif z >= 0.4:
            q2 = 1.3788  # Tilt all the way up
        else:
            q2 = (z / 0.4) * 1.3788 - 1.55  # Linear interpolation

        # Clamp the joint angle to its limits
        q2 = self.clamp(q2, math.radians(-103), math.radians(90))

        # Elbow joint (Joint 3)
        distance_xy = math.sqrt(x**2 + y**2)
        q3 = self.clamp(distance_xy / 0.5, math.radians(-53), math.radians(79))  # Assuming 0.5m is the max reach

        # Wrist adjustment (Joint 4)
        q4 = -q2  # Keep the claw perpendicular to the ground

        # Clamp the wrist adjustment to its limits
        q4 = self.clamp(q4, math.radians(-100), math.radians(117))

        print(f"Moving arm to: q1: {q1}, q2: {q2}, q3: {q3}, q4: {q4}")

        # Directly set the joint positions
        self.move_group_arm.set_joint_value_target([q1, q2, q3, q4])
        self.move_group_arm.go(wait=True)
        self.move_group_arm.stop()

        # Adding sleep to prevent abort control failed
        rospy.sleep(0.1)

    def hand_state_callback(self, data):
        """Callback function for controlling gripper based on hand state"""
        is_open = data.data
        if is_open != self.gripper_open:
            if is_open:
                print("Opening gripper as hand is open")
                self.move_group_gripper.go([0, 0], wait=True)  # Open gripper
            else:
                print("Closing gripper as hand is closed")
                self.move_group_gripper.go([-0.01, -0.01], wait=True)  # Close gripper
            self.move_group_gripper.stop()
            self.gripper_open = is_open

    def run(self):
        rospy.spin()

    def clamp(self, n, smallest, largest):
        """Helper function to clamp a value to a range"""
        return max(smallest, min(n, largest))

if __name__ == '__main__':
    node = ArmControl()
    node.run()



