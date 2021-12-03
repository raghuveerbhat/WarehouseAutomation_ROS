#!/usr/bin/python3
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
import time

class NavigationStack():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move base server")
        self.client.wait_for_server()
        self.landmarks = [[0.963087797164917, 0.27393558621406555, 0, 0, 0, 0.026384433758031428, 0.9996518702304658],
                            [1.2564698457717896, -6.91493034362793, 0, 0, 0, 0.02384949077767648, 0.9997155604418917]]
        self.set_goal()

    def set_goal(self):
        for i, landmark in enumerate(self.landmarks):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = landmark[0]
            goal.target_pose.pose.position.y = landmark[1]
            goal.target_pose.pose.position.z = landmark[2]
            goal.target_pose.pose.orientation.x = landmark[3]
            goal.target_pose.pose.orientation.y = landmark[4]
            goal.target_pose.pose.orientation.z = landmark[5]
            goal.target_pose.pose.orientation.w = landmark[6]
            self.client.send_goal(goal)
            self.client.wait_for_result()
            print("GOAL {} REACHED".format(i))
            time.sleep(5)



if __name__ == '__main__':
    rospy.init_node("navigation_node")
    node = NavigationStack()
    rospy.spin()
