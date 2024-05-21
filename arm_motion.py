#!/usr/bin/python3

import rospy
import sys
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import moveit_commander

ONE_RAD = 2*math.pi
JOINT_1_MAX_MOVEMENT = ONE_RAD * 0.1
JOINT_2_MAX_MOVEMENT = ONE_RAD * 0.1
JOINT_3_MAX_MOVEMENT = ONE_RAD * 0.1
JOINT_4_MAX_MOVEMENT = ONE_RAD * 0.1


JOINT_2_MIN = -103
JOINT_2_MAX = 90

JOINT_3_MIN = -53
JOINT_3_MAX = 79

JOINT_4_MIN = -100
JOINT_4_MAX = 117

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
        self.current_joint_positions = {
            "joint1": (0, 0),
            "joint2": 0,
            "joint3": 0,
            "joint4": 0
        }

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

    def _get_theta(self, vector_a, vector_b):
        dot_product = vector_a[0] * vector_b[0] + vector_a[1] * vector_b[1]
        magnitude_a = math.sqrt(vector_a[0]**2 + vector_a[1]**2)
        magnitude_b = math.sqrt(vector_b[0]**2 + vector_b[1]**2)
        return math.acos(dot_product / (magnitude_a * magnitude_b))
    
    def _rotate_vector(vector, angle_change_rad):
        x = vector[0] * math.cos(angle_change_rad) - vector[1] * math.sin(angle_change_rad)
        y = vector[0] * math.sin(angle_change_rad) + vector[1] * math.cos(angle_change_rad)
        print("new clamped vector: ", (x, y))
        return (x, y)
    
    def _get_joint_1(self, x, y):
        current = self.current_joint_positions["joint1"]
        rad = self._get_theta(current, (x, y))
        final_rad = self.clamp(rad, -JOINT_1_MAX_MOVEMENT, JOINT_1_MAX_MOVEMENT)

        # update state
        self.current_joint_positions["joint1"] =self._rotate_vector(current, final_rad)
        return final_rad        

    
    def _get_joint_2(self, z):
        # # Base tilt (Joint 2)
        # if z <= 0:
        #     q2 = math.radians(-103)  # Tilt all the way down
        # elif z >= 0.4:
        #     q2 = math.radians(90)  # Tilt all the way up
        # else:
        #     q2 = math.radians(-103) + (z / 0.4) * (math.radians(90) - math.radians(-103))  # Linear interpolation

        # # Clamp the joint angle to its limits
        # q2 = self.clamp(q2, math.radians(-103), math.radians(90))
        return 0
    
    def _get_joint_3(self, x, y):
        # Elbow joint (Joint 3)
        # distance_xy = math.sqrt(x**2 + y**2)
        # q3 = (distance_xy / 0.5) * math.radians(79)  # Assuming 0.5m is the max reach
        # q3 = self.clamp(q3, math.radians(-53), math.radians(79))
        return 0
    
    def _get_joint_4(self, z):
        # Wrist adjustment (Joint 4)
        # q4 = -q2  # Keep the claw perpendicular to the ground
        # q4 = self.clamp(q4, math.radians(-100), math.radians(117))
        return 0
    

    def hand_position_callback(self, data):
        """Callback function for controlling arm based on hand position"""
        x, y, z = data.point.x, data.point.y, data.point.z

        q1 = self._get_joint_1(x, y)
        q2, q3, q4 = 0, 0, 0
        # q2 = self._get_joint_2(z)
        # q3 = self._get_joint_3(x, y)
        # q4 = self._get_joint_4(z)

        print(f"Moving arm to: q1: {q1}, q2: {q2}, q3: {q3}, q4: {q4}")

        new_pos = [q1, q2, q3, q4]
        self.move_group_arm.set_joint_value_target(new_pos)
        plan_success, trag_msg, planning_time, err_code = self.move_group_arm.plan(wait=True)
        if not plan_success:
            print("error in planning, skipping execution")
            rospy.logwarn(f"Motion planning failed in {planning_time} sec. Check for potential issues with the target joint values or robot state. Error code: {err_code}, {trag_msg}")
            return
        success = self.move_group_arm.go(wait=True)
        self.move_group_arm.stop()

        if not success:
            rospy.logwarn("Motion planning failed during execution. Check for potential issues with the target joint values or robot state.")

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