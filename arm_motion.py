#!/usr/bin/python3

import rospy
import sys
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import moveit_commander
import time

# Configuring frame rate
FORCED_FRAME_RATE = 0.25
ONE_RAD = 2*math.pi

# Configuring max movement for each joint per frame
JOINT_1_MAX_MOVEMENT = ONE_RAD * 0.03
JOINT_2_MAX_MOVEMENT = ONE_RAD * 0.008
JOINT_3_MAX_MOVEMENT = ONE_RAD * 0.01
JOINT_4_MAX_MOVEMENT = ONE_RAD * 0.1

# Configuring max range of motion for joint 1
JOINT_1_MIN = -math.pi 
JOINT_1_MAX = math.pi 

# Configuring max range of motion for joint 2
JOINT_2_MIN = -1 # up
JOINT_2_MAX = math.pi / 2 # down
JOINT_2_MIN_M = 0.0
JOINT_2_MAX_M = 0.3

# Configuring max range of motion for joint 3
JOINT_3_MIN = 0 # at 90
JOINT_3_MAX = 0.9  # the max as specified by spec
JOINT_3_MIN_M = 0.0
JOINT_3_MAX_M = 0.2

class ArmControl(object):
    """
    Class for controlling the arm and gripper based on hand position and state
    """
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
        self.move_group_arm.set_max_velocity_scaling_factor(1)
        self.move_group_arm.set_max_acceleration_scaling_factor(1)
        rospy.sleep(3) 

        # Reset gripper position
        gripper_joint_open = [0.01, 0.01]
        self.move_group_gripper.go(gripper_joint_open, wait=True)
        self.move_group_gripper.stop()
        self.gripper_open = True
        rospy.sleep(3)

        # get current time
        self.last_action = time.time()
        self.last_action_hand = time.time()
        self.last_hand_preds = []
        
        # Set subscribers
        rospy.Subscriber("hand_center", PointStamped, self.hand_position_callback)
        rospy.Subscriber("hand_open", Bool, self.hand_state_callback)

    def _map_distance_to_radians(self, distance_m, joint_min_m, joint_max_m, joint_max):
        """
        Maps a distance in meters to a range of radians
        """
        if  distance_m <= joint_min_m: # down
            return math.pi / 2
        elif  distance_m >= joint_max_m: # up
            return 0
        else:
            # interpolation
            return (1 - (distance_m / joint_max_m)) * (joint_max)
    

    def _get_joint_1(self, x, y): # swivel
        """
        The swivel servo, mapping the x,y hand position to a 0 - 180 degree range (0 - pi radians)
        """
        hand_rad = math.atan2(y, x)  # get radians of hand position
        hand_rad = hand_rad - math.pi / 2
        if (hand_rad < 0):
            hand_rad += 2 * math.pi
        
        if (hand_rad > JOINT_1_MAX):
            hand_rad = hand_rad - 2 * math.pi
        
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
        
        print(f"[JOINT 1] j1_rad {j1_rad:.4f} xy ({x:.4f},{y:.4f}), hand_rad: {hand_rad:.4f}, rad_delta: {rad_delta:.4f}, rad_delta_clamped: {rad_delta_clamped:.4f}, target_rad {target_rad:.4f} final_target_rad: {final_target_rad:.4f}",
        file=sys.stderr)
        return final_target_rad        


    def _get_joint_2(self, z): 
        """
        The tilt servo, mapping the distance from the ground (meters) to a 0 - 90 degree range (0 - pi/2 radians)
        """
        hand_rad = self._map_distance_to_radians(z, JOINT_2_MIN_M, JOINT_2_MAX_M, JOINT_2_MAX) - 0.5
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
        print(f"[JOINT 3] j3_rad {j3_rad:.4f} distance {distance_from_origin}, hand_rad: {hand_rad:.4f}, rad_delta: {rad_delta:.4f}, rad_delta_clamped: {rad_delta_clamped:.4f}, target_rad {target_rad:.4f} ")
        return -target_rad
    
    def _get_joint_4(self, q2):
        """
        NoOp, the wrist joint
        """
        return 0
    

    def hand_position_callback(self, data):
        """Callback function for controlling arm based on hand position"""
        x, y, z = data.point.x, data.point.y, data.point.z
        
        # only perform an action every 0.25 seconds
        # This is because the camera is running at 30ps and we need to throttle the frames to prevent
        # moveit from crashing/aborting
        now = time.time()
        if now - self.last_action < FORCED_FRAME_RATE:
            return
        self.last_action = now

        # get current joint values
        curr_j1, curr_j2, curr_j3, curr_j4 = self.move_group_arm.get_current_joint_values()
        print(f"Current Location {curr_j1:.4f},{curr_j2:.4f}, {curr_j3:.4f}, {curr_j4:.4f}", file=sys.stderr)
        
        q1, q2, q3, q4 = 0,0,0,0 # init (for test as well)
        q1 = self._get_joint_1(x, y)
        q2 = self._get_joint_2(z)
        q3 = self._get_joint_3(x, y)
        q4 = self._get_joint_4(q2)

        print(f"Moving arm to: q1: {q1}, q2: {q2}, q3: {q3}, q4: {q4}", file=sys.stderr)
        if q1 == q2 == q3 == 0: # prevent any client side abort freakouts
            return
        new_pos = [q1, q2, q3, q4] # new joint positions for the arm
        success = self.move_group_arm.go(new_pos, wait=False) # move the arm
        self.move_group_arm.stop()

        if not success:
            rospy.logwarn("Motion planning failed during execution. Check for potential issues with the target joint values or robot state.")

        # Tiny sleep to further prevent moveit from crashing, important
        rospy.sleep(0.1)

    def hand_state_callback(self, data):
        """
        Callback function for controlling gripper based on hand state
        """

        # only perform an action every 0.25 seconds, throttling to prevent moveit from crashing
        now = time.time()
        if now - self.last_action_hand < FORCED_FRAME_RATE:
            return
        self.last_action_hand = now

        # smoothing movement by basing on the last 2 values published from the hand_open topic
        raw_is_open = data.data
        self.last_hand_preds.append(raw_is_open)
        self.last_hand_preds = self.last_hand_preds[-2:]
        is_open = sum(self.last_hand_preds) > 1

        # open or close gripper based on hand state
        if is_open:
            self.move_group_gripper.go([0.01, 0.01], wait=True)  # Open gripper
        else:
            self.move_group_gripper.go([-0.01, -0.01], wait=True)  # Close gripper
        self.move_group_gripper.stop()

    def run(self):
        """
        Keep things running
        """
        rospy.spin()

    def clamp(self, n, smallest, largest):
        """
        Helper function to clamp a value to a range
        """
        return max(smallest, min(n, largest))

if __name__ == '__main__':
    node = ArmControl()
    node.run()