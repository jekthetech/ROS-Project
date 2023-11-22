#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, BoundingBox2DArray
import cv2
import numpy as np
import pdb
import struct
import ctypes
import math



################ POINT FILTER ##################################

class point_filter:

    def __init__(self):   
        #Initialize variables:
        self.all_raw_bbox_points =[]
        self.raw_bbox_points = []
        self.final_points =[]

        #PUBLISHERS/SUBSCRIBERS MAY NEED TO BE MODIFIED TO REFLECT THE CORRECT TOPICS
        bbox_sub = rospy.Subscriber('/Camera/Results', Detection2DArray, bounding_box_callback)

        #Not needed for transformations
        #image_sub = rospy.Subscriber('/camera_throttled/depth/color/points', PointCloud2, point_cloud_callback)

        

    def publish_points(self):
        point_pub = rospy.Publisher("/Camera/Results", PointStamped)
        point_pub.publish(self.final_points)
        


    # Extract bounding box dimensions and convert
    def bounding_box_callback(self,data):

        all_filtered_points = []

        bbox_time = data.header.stamp
       
        for detection in data.detections:
            # access the bounding box points
            bbox = data.detections.bbox
            
            width = bbox.size_x
            height = bbox.size_y
            bbox_center_x = bbox.center.x
            bbox_center_y = bbox.center.y

          

            xMin = bbox_center_x - width/2
            xMax = bbox_center_x + width/2
            yMin = bbox_center_y - height/2
            yMax = bbox_center_y + height/2

            point_to_grab=PointStamped()
            3D_bbox_points = []
    
            row=0
            col=0
            for row in range(yMin,yMax):
                for col in range(xMin,xMax):
                    index = (row * data.row_step) + (col * data.point_step)
                    # Get the XYZ points [meters]
                    (X, Y, Z,rgb) = struct.unpack_from('fffl', data.data, offset=index)

                    #create point stamped object to use when transformiing points:
                    3D_point =PointStamped()

                    #frame will eventually be 'usb_cam/image_raw'
                    3D_point.header.frame_id = 'camera_color_optical_frame'
                    3D_point.header.stamp = bbox_time

                    3D_point.point.x = X
                    3D_point.point.y = Y
                    3D_point.point.z = Z

                    # Append to array of 3D points in camera frame:
                    3D_bbox_points.append(3D_point)

            #Transfrom 3d points to map frame
            #transformation info:
            curr_time = rospy.time(0)
            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer);

            try:
                #from frame will be 'usb_cam/image_raw'
                transform = tfBuffer.lookup_transform(
                target_frame = 'map',
                target_time = curr_time,
                source_frame = 'camera_color_optical_frame',
                source_time = bbox_time,
                fixed_frame - 'base_link',
                timeout=rospy.duration(1.0))

                transformed_points = tfBuffer.transform(3D_bbox_points, 'base_link', rospy.Duration(1.0))
                # Z height sorting and filtering clusters into a single point
                point_to_grab = filter(transformed_points)

                all_filtered_points.append(point_to_grab)
                        


            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("error making transformation")
                rate.sleep()
                continue

        #These are the points that will be published
        self.final_points = cluster_points(all_filtered_points)



    ######### FUNCTIONS USED IN THE BOUNDING BOX CALLBACK ##########
    def filter_points(self, points):
        #filters all 3D points within one bounding box, and returns the highest point

        #assuming floor height is 0.0 Confirm with team.
        #to use to filter out floor points
        floor_height=0.0

        #making max tshirt height 5 inches off ground. Confirm with team
        max_tshirt_height = 0.127

        #will return the highest point in the filtered array
        final_point=PointStamped(0,0,0)

        for point in points:
            #(if the point is not the floor and under 5 inches)
            if point.point.z>floor_height and point.point.z < max_tshirt_height:
                if point.point.z > final_point.point.z:
                    final_point = point
    
        return final_point


    def cluster_points(self, point_array):

        point_arr=point_array

        #THRESHOLD TO BE MODIFIED WITH ACTUAL DATA
        threshold = 5

        #go through all points, and combine any that are within the threshold of eachother:
        for i in range(len(point_arr)):
            #point to compare:
            current_point = point_arr[i]

            #array will hold all points that are close to eachother:
            points_to_merge=[]
            locations =[]
            #go through the rest of the points:
            for j in range(i+1, len(point_arr)):
                difference = find_distance(current_point, point_arr[j])
                if difference < threshold:
                    points_to_merge.append(point_arr[j])
                    locations.append(j)

            if len(points_to_merge) > 0:
                updated_point = find_average(current_point, points_to_merge)
                for index in locations:
                    point_arr[index]=updated_point

        #remove multiples:
        final_points=list(set(point_arr))

        #returns points 
        return final_points
            
 
    def find_distance(self, point1, point2):
        x1, y1, z1 = point1.point.x, point1.point.y, point1.point.z
        x2, y2, z2 = point2.point.x, point2.point.y, point2.point.z

        #differences:
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1

        #distance formula:
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        return distance



    def find_average(:self, current_point, point_arr):
        #sums
        sum_x=current_point.point.x
        sum_y=current_point.point.y
        sum_z=current_point.point.z

        for point in point_arr:
            sum_x += point.point.x
            sum_y += point.point.y
            sum_z += point.point.z

        avg_x = sum_x /len(points)
        avg_y = sum_y /len(points)
        avg_z = sum_z /len(points)

        avg_point = PointStamped(point.header, Point(avg_x, avg_y, avg_z)

        return avg_point


    

if __name__ == '__main__':
    filtered_points = point_filter()
    filtered_points.publish_points()


