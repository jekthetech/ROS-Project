#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 2.7
    goal.target_pose.pose.position.y = 0.5
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
    

def trigger_head_scan():
    rospy.wait_for_service('/funmap/trigger_head_scan')
    try:
        trigger_scan = rospy.ServiceProxy('/funmap/trigger_head_scan', Trigger)
        response = trigger_scan()
        return response.message
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

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
        movebase_client()
        trigger_head_scan()
        # result = send_goal_to_topic(2)
        # rospy.loginfo("1 Goal execution done!")
        # rospy.sleep(15)
        # result = send_goal_to_topic(1)
        # rospy.loginfo("2 Goal execution done!")
        # rospy.sleep(15)
        # result = send_goal_to_topic(3)
        # rospy.loginfo("Home Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
