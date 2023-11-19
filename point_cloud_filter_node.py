#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
#from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, BoundingBox2DArray
import cv2
import numpy as np
import pdb
import struct
import ctypes
from perception_msgs.msg import Detection2DArray


#Initialize ROS node
rospy.init_node('point_cloud_filter')

def point_cloud_callback():
    #(global) raw_bbox_points =[xmin, ymin, xmax,  ymax]
    # bbox globals
    xMin = raw_bbox_points[0]
    xMax = raw_bbox_points[2]
    yMin = raw_bbox_points[1]
    yMax = raw_bbox_points[3]

    # bbox pixels to 3D points
    
    xyz_image=np.zeros((yMax-yMin,xMax-xMin,3),np.float32)
    
    row=0
    col=0
    for row in range(yMin,yMax):
        for col in range(xMin,xMax):
            index = (row*msg.row_step) + (col*msg.point_step)
            # Get the XYZ points [meters]
            (X, Y, Z,rgb) = struct.unpack_from('fffl', msg.data, offset=index)
            xyz_image[row-yMin,col-xMin,0]=X
            xyz_image[row-yMin,col-xMin,1]=Y
            xyz_image[row-yMin,col-xMin,2]=Z

            #create point stamped object to use when transformiing points:
            3D_point =PointStamped()

            #frame will eventually be 'usb_cam/image_raw'
            3D_point.header.frame_id = 'camera_color_optical_frame'
            3D_point.header.stamp = rospy.Time()

            3D_point.point.x = X
            3D_point.point.y = Y
            3D_point.point.z = Z

            # Append to array of 3D points in camera frame:
            3D_bbox_points.append(3D_point)
    
    #Transfrom 3d points to map frame
    #transformation info:
    curr_time = rospy.Time(0)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer);

    try:
        #from frame will be 'usb_cam/image_raw'
        transform = tfBuffer.lookup_transform('camera_color_optical_frame','base_link', curr_time, rospy.Duration(1.0))
        transformed_points = tfBuffer.transform(3D_bbox_points, 'base_link', rospy.Duration(1.0))    

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue

            

    # Z height sorting and filtering clusters into a single point
    point_to_grab = filter(transformed_points)
    
    # Publish the Points Array - One Point for every cloth

    return


#all of the following has been moved to the main function for testing. this should be called one testing complete
def filter_points(points):
    
    #assuming floor height is 0.0 Confirm with team.
    #to use to filter out floor points
    floor_height=0.0

    #making max tshirt height 5 inches off ground. Confirm with team
    max_tshirt_height = 0.127

    #will return the highest point in the filtered array
    final_point=PointStamped(0,0,0)

    for point in points:
        #(if the point is not the floor and under 5 inches)
        if point.z>floor_height and point.z < max_tshirt_height:
            if point.z > max_point.z:
                final_point = point
    
    return final_point
      


# Extract bounding box dimensions
def bounding_box_callback(data):
    # access the bounding box points
    bbox = data.bbox
            
    width = bbox.size_x
    height = bbox.size_y
    bbox_center_x = bbox.center.x
    bbox_center_y = bbox.center.y

    '''just in case:
    top_left_corner = [bbox_center_x - width/2, bbox_center_y - height /2]
    top_right_corner = [bbox_center_x + width/2, bbox_center_y - height/2]
    bottom_left_corner = [bbox_center_x + width/2, bbox_center_y + height/2]
    bottom_right_corner =[bbox_center_x - width/2, bbox_center_y + height]
        '''

    xmin = bbox_center_x - width/2
    xmax = bbox_center_x + width/2
    ymin = bbox_center_y - height/2
    ymax = bbox_center_y + height/2

    raw_bbox_points =[xmin, ymin, xmax,  ymax]
        # TODONE: xy min/max - globals
    return


#currently processes one node at a time.
#def find_cloth_points():
    # Take in the array of cloth
    # return the list


#print("Outside of while loop.")
while not rospy.is_shutdown():


    #print("Inside of loop")
    #Subscribe to topic with detected bounding boxes from perception node
    #from perception node: bounding_box_publisher = rospy.Publisher("/Camera/Results", Detection2DArray)
    #raw_bbox_points = xmin, ymin, xmax,  ymax
    raw_bbox_points = []
    3D_bbox_points = []

    point_to_grab=PointStamped()

    bbox_sub = rospy.Subscriber('/Camera/Results', Detection2DArray, bounding_box_callback)
    
    image_sub = rospy.Subscriber('/camera_throttled/depth/color/points', PointCloud2, point_cloud_callback)

    point_pub = rospy.Publisher("/Camera/Results", PointStamped)
    point_pub.publish(point_to_grab)

    

    rospy.spin()
