#!/usr/bin/python3

import rospy
import sys
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import moveit_commander
import time
FORCED_FRAME_RATE = 0.5

ONE_RAD = 2*math.pi
JOINT_1_MAX_MOVEMENT = ONE_RAD * 0.05
JOINT_2_MAX_MOVEMENT = ONE_RAD * 0.1
JOINT_3_MAX_MOVEMENT = ONE_RAD * 0.1
JOINT_4_MAX_MOVEMENT = ONE_RAD * 0.1

JOINT_1_MIN = -math.pi 
JOINT_1_MAX = math.pi 

JOINT_2_MIN = 0 # up
JOINT_2_MAX = math.pi / 2 # down
JOINT_2_MIN_M = 0.0
JOINT_2_MAX_M = 0.7

JOINT_3_MIN = 0 # at 90
JOINT_3_MAX = math.pi /2  # at 180, fully extended
JOINT_3_MIN_M = 0.0
JOINT_3_MAX_M = 0.5

class ArmControl(object):
    def __init__(self):
        rospy.init_node("arm_control")

        # Initialize arm
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        arm_joint_positions = [0, 0, 0, 0]
        self.move_group_arm.go(arm_joint_positions, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(3) 

        # Reset gripper position
        gripper_joint_open = [0, 0]
        self.move_group_gripper.go(gripper_joint_open, wait=True)
        self.move_group_gripper.stop()
        self.gripper_open = True
        rospy.sleep(3)

        # get current time
        self.last_action = time.time()
        
        # Set subscribers
        rospy.Subscriber("hand_center", PointStamped, self.hand_position_callback)
        rospy.Subscriber("hand_open", Bool, self.hand_state_callback)

    def _map_distance_to_radians(z_in_meters, joint_min_m, joint_max_m, joint_max):
        if  z_in_meters <= joint_min_m: # down
            return math.pi / 2
        elif  z_in_meters >= joint_max_m: # up
            return 0
        else:
            # interpolation
            return (1 - (z_in_meters / joint_max_m)) * (joint_max)
    

    # TODO these could be combined in a single function, but keep the separate for testing
        

    def _get_joint_1(self, x, y): # swivel
        """
        The swivel servo, mapping the x,y hand position to a 0 - 180 degree range (0 - pi radians)
        """
        hand_rad = math.atan2(y, x) # get radians of hand position
        j1_rad, _, _, _ = self.move_group_arm.get_current_joint_values() # get current joint values (swivel)
        rad_delta = hand_rad - j1_rad # get the difference between the hand and current joint values
        rad_delta_clamped = self.clamp(rad_delta, -JOINT_1_MAX_MOVEMENT, JOINT_1_MAX_MOVEMENT) # clamp the difference to the max movement allowed
        target_rad = j1_rad + rad_delta_clamped # add the clamped difference to the current joint value to get the target joint value
        
       
        # check bounds, ensure that the target joint value is within the allowed range
        final_target_rad = target_rad
        if target_rad > JOINT_1_MAX:
            final_target_rad = JOINT_1_MAX
        elif target_rad < JOINT_1_MIN:
            final_target_rad = JOINT_1_MIN
        
        print(f"[JOINT 1] j1_rad {j1_rad:.4f} xy ({x:.4f},{y:.4f}), hand_rad: {hand_rad:.4f}, rad_delta: {rad_delta:.4f}, rad_delta_clamped: {rad_delta_clamped:.4f}, target_rad {target_rad:.4f} final_target_rad: {final_target_rad:.4f}")
        return final_target_rad        


    def _get_joint_2(self, z): 
        """
        The tilt servo, mapping the distance from the ground (meters) to a 0 - 90 degree range (0 - pi/2 radians)
        """
        hand_rad = self._map_distance_to_radians(z, JOINT_2_MIN_M, JOINT_2_MAX_M, JOINT_2_MAX)
        _, j2_rad, _, _ = self.move_group_arm.get_current_joint_values()
        rad_delta = hand_rad - j2_rad
        rad_delta_clamped = self.clamp(rad_delta, -JOINT_2_MAX_MOVEMENT, JOINT_2_MAX_MOVEMENT)
        target_rad = j2_rad + rad_delta_clamped
        print(f"[JOINT 2] j2_rad {j2_rad:.4f} z {z:.4f}, hand_rad: {hand_rad:.4f}, rad_delta: {rad_delta:.4f}, rad_delta_clamped: {rad_delta_clamped:.4f}, target_rad {target_rad:.4f} ")
        return target_rad
    
    def _get_joint_3(self, x, y): 
        """
        The elbow joint, mapping the distance from the origin to a 0 - 90 degree range (0 - pi/2 radians)
        """
        distance_from_origin = abs(math.sqrt(x**2 + y**2))
        hand_rad = self._map_distance_to_radians(distance_from_origin, JOINT_3_MIN, JOINT_3_MAX, JOINT_3_MAX)
        _, _, j3_rad, _ = self.move_group_arm.get_current_joint_values()
        rad_delta = hand_rad - j3_rad
        rad_delta_clamped = self.clamp(rad_delta, -JOINT_3_MAX_MOVEMENT, JOINT_3_MAX_MOVEMENT)
        target_rad = j3_rad + rad_delta_clamped
        print(f"[JOINT 3] j3_rad {j3_rad:.4f} xy ({x:.4f},{y:.4f}), hand_rad: {hand_rad:.4f}, rad_delta: {rad_delta:.4f}, rad_delta_clamped: {rad_delta_clamped:.4f}, target_rad {target_rad:.4f} ")
        return target_rad
    
    def _get_joint_4(self, q2):
        # We dont need this? (inverse of the tilt, but the elbow makes this more complicated)
        return -q2
    

    def hand_position_callback(self, data):
        """Callback function for controlling arm based on hand position"""
        x, y, z = data.point.x, data.point.y, data.point.z
        now = time.time()

        # only publish an action every 0.5 seconds
        if now - self.last_action < FORCED_FRAME_RATE:
            return
        self.last_action = now
        curr_j1, curr_j2, curr_j3, curr_j4 = self.move_group_arm.get_current_joint_values()
        print(f"Current Location {curr_j1:.4f},{curr_j2:.4f}, {curr_j3:.4f}, {curr_j4:.4f}")

        q1 = self._get_joint_1(x, y)
        q2, q3, q4 = 0, 0, 0
        # q2 = self._get_joint_2(z)
        # q3 = self._get_joint_3(x, y)
        # q4 = self._get_joint_4(q2)

        print(f"Moving arm to: q1: {q1}, q2: {q2}, q3: {q3}, q4: {q4}")

        new_pos = [q1, q2, q3, q4]
        self.move_group_arm.set_joint_value_target(new_pos)
        success = self.move_group_arm.go(wait=True)
        self.move_group_arm.stop()

        if not success:
            rospy.logwarn("Motion planning failed during execution. Check for potential issues with the target joint values or robot state.")

        rospy.sleep(0.1)

    def hand_state_callback(self, data):
        """Callback function for controlling gripper based on hand state"""
        is_open = data.data
        if is_open != self.gripper_open:
            if is_open:
                self.move_group_gripper.go([0, 0], wait=True)  # Open gripper
            else:
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