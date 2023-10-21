#Point cloud filterning node that takes in point cloud data from perception node and filters it

#!/usr/bin/env python3

# Libraries:
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2 #PointCloud2 allows for n-dimensional data, and can return messages with height/width values
#(so 2d structures can be used with images, which helps with object detection in our project.)
from sensor_msgs.msg import PointCloud2 #This is needed to use Point Cloud 2 as well
from geometry_msgs.msg import Point


# Initialize:
rospy.init_node('filter_point_cloud_node');

# Creates publisher to publish filtered points. psuedo Topic = "/filtered", PointCloud2 type, queue size is 10 (not exactly sure what else it should be in this case.)
pub = rospy.Publisher('/filtered_points', PointCloud2, queue_size=10)

# Callback function for the point cloud subscriber
def callback(msg):
    # Converts point cloud data to a list of points
    point_list = np.array(pc2.read_points)

    # Create a list to store filtered points
    filtered_points = []

    # Loop through all points in the point cloud
    for point in point_list:
        #not sure what to filter the points on. For now, just used a threshold of 0. This should be modified in the future
        if point[0] > 0:
            # Creates a Point object for each filtered point, sets the x,y,and z values
            filtered_point = Point()
            filtered_point.x = point[0]
            filtered_point.y = point[1]
            filtered_point.z = point[2]
            # And adds the filtered points to a list
            filtered_points.append(filtered_point)

    # Publishes the filtered points cloud as a PointCloud2 message
    pub.publish(pc2.create_cloud(msg.header, msg.fields, filtered_points))

# Subscribe to a point cloud topic
sub = rospy.Subscriber('/object_point_cloud', PointCloud2, callback)

# Run the node
rospy.spin() 
