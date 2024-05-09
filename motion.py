
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

        # Publish movement
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

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

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = ArmMotion()
    node.run()
