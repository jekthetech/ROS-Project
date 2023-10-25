#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def send_goal_to_topic(x):
    print("publisher set")
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    if x == 1:
        goal.pose.position.x = 3.0
        goal.pose.position.y = 2.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.0
    elif x == 2:
        goal.pose.position.x = 3.5
        goal.pose.position.y = 2.5
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.0
    else:
        goal.pose.position.x = 4.0
        goal.pose.position.y = 4.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.0

    pub.publish(goal)
    print("Goal sent!")

if __name__ == '__main__':
    try:
        print("Starting points!")
        rospy.init_node('movebase_client_py')
        result = send_goal_to_topic(2)
        rospy.loginfo("1 Goal execution done!")
        rospy.sleep(15)
        result = send_goal_to_topic(1)
        rospy.loginfo("2 Goal execution done!")
        rospy.sleep(15)
        result = send_goal_to_topic(3)
        rospy.loginfo("Home Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
