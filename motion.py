
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

        # Set length of arm parts between the joints in centimeters
        self.l_1 = 13.5
        self.l_2 = 12.9

        # We will set joint angles q_1 and q_2
        self.q_1 = 0
        self.q_2 = 0

    def xyz_callback(self, data):
        """ Callback function for calculating inverse kinematics of the
        arm based on x, y, z input in a Twist message """
        print("Make arm movements based on twist message")
        x, y, z = data.linear.x, data.linear.y, data.linear.z
        print("Got x y z: ", x, y, z)

        # Squared distance
        sq_dist = x**2 + y**2

        # Find joint angle q2
        tmp = (self.l_1 ** 2) + (self.l_2 ** 2) - sq_dist
        cos_alpha = tmp / (2 * self.l_1 * self.l_2)
        if cos_alpha > 1 or cos_alpha < 1:
            print("Cosine of alpha out of bounds")
            cos_alpha = clamp(cos_alpha, -1, 1)

        alpha = math.acos(cos_alpha)
        q_2 = math.pi - alpha

        # Find joint angle q1
        tmp_2 = math.atan(y/x)
        # TODO: make sure atan is valid
        tmp_3 = (self.l_2 * math.sin(q_2)) / (self.l_1 + (self.l_2 * math.cos(q_2)))
        tmp_3 = math.atan(tmp_3)

        q_1 = tmp_2 - tmp_3

        self.q_1 = q_1
        self.q_2 = q_2

        print("Found q_1, q_2: ", self.q_1, self.q_2)

    def run(self):
        # Keep the program alive.
        rospy.spin()

def clamp(n, smallest, largest):
        """ Helper function to clamp a value to a range """
        return max(smallest, min(n, largest))

if __name__ == '__main__':
    # Declare a node and run it.
    node = ArmMotion()
    node.run()
