
import rospy
import math
import moveit_commander

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class ArmMotion(object):


    def __init__(self):
        # Start rospy node.
        rospy.init_node("arm_motion")

        # Set a subscriber to twist
        rospy.Subscriber("/cmd_vel", Twist, self.xyz_callback)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0, 0, 0, 0], wait=True)
        self.move_group_arm.stop()

        rospy.sleep(3) # give the arm time to perform the action before moving the gripper

        # Reset gripper position
        gripper_joint_open = [0, 0]
        self.move_group_gripper.go(gripper_joint_open, wait=True)
        self.move_group_gripper.stop()

        rospy.sleep(3)

    def xyz_callback(self, data):
        print("Make arm movements based on twist message")

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = ArmMotion()
    node.run()
