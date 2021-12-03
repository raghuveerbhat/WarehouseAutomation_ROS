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
        self.landmarks = [[1.4473416805267334, 0.41364067792892456, 0.0, 0.0, 0.0, -0.011503674603633503, 0.9999338305461085],
                            [1.0017170906066895, -1.3638901710510254, 0.0, 0.0, 0.0, 0.010521600528978107, 0.9999446464291454],
                            [0.9813948273658752, -3.1724913120269775, 0.0, 0.0, 0.0, 0.0013030230571348166, 0.999999151065096],
                            [0.9532657861709595, -4.968116283416748, 0.0, 0.0, 0.0, -0.0022372289058545302, 0.9999974974002799],      
                            [1.4260408878326416, -6.819857120513916, 0.0, 0.0, 0.0, -0.0015038960997956234, 0.9999988691476211],      
                            [1.5627377033233643, -8.728625297546387, 0.0, 0.0, 0.0, -0.02648123511897974, 0.9996493106017597]]
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
