#!/usr/bin/env python3

"""
Code: to detect colored cubes using a camera and publish their positions in the world frame.

-> gets object's pixel positon.
-> converts the pixel position to camera frame.
-> transforms the camera frame to world frame using TF.
-> publishes the cube position in the world frame.
-> The code uses OpenCV for image processing and ROS for communication.

Command to view the camera feed manually:
-     command $: rosrun image_view image_view image:=/camera_top/image_raw

Command to view the camera parameters i.e, Camera Calibration parameters: 
-     command $: rostopic echo /camera_top/camera_info    ($<camera name> = camera_top)
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from ur5_control.msg import Cube, CubeArray  # Importing custom message

# Initializing OpenCV bridge
bridge = CvBridge()

# Initializing TF buffer and listener
tf_buffer = None
tf_listener = None

""" Callback function: to capture images with the camera and detect colored cubes """
def image_callback(msg):
    global tf_buffer
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")            # Convert ROS image message to OpenCV image

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)   # HSV color space conversion

        # Defining color ranges:
        color_ranges = {
            "red": ([0, 100, 100], [10, 255, 255]),
            "green": ([35, 100, 100], [85, 255, 255]),
            "blue": ([100, 100, 100], [130, 255, 255])
        }
        
        cube_array_msg = CubeArray()                # an array message to store multiple cubes

        # Looping through each color and detect cubes
        for color_name, (lower, upper) in color_ranges.items():
            # Create a mask for the color
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(hsv_image, lower, upper)

            # Find contours of the detected color
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                
                x, y, w, h = cv2.boundingRect(contour)      # Get the bounding box of the contour

                # Calculate the centroid of the bounding box:
                centroid_x = x + w // 2
                centroid_y = y + h // 2

                # Drawing the bounding box and centroid on the image
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(cv_image, (centroid_x, centroid_y), 5, (0, 0, 255), -1)

                # Add a background rectangle for the text label
                text = f"{color_name}"
                (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
                cv2.rectangle(cv_image, (x, y - text_height - 10), (x + text_width, y), (255, 255, 255), -1)

                cv2.putText(cv_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
                
                print("-------------")
                # Log the centroid position of the cube
                rospy.loginfo(f"{color_name.capitalize()} cube centroid (pixel): ({centroid_x}, {centroid_y})")

                # Transform pixel coordinates to 3D world coordinates using fixed depth
                cube_position_world = transform_pixel_to_world(centroid_x, centroid_y)

                if cube_position_world:
                    rospy.loginfo(f"{color_name.capitalize()} cube position (world): ({cube_position_world.point.x}, {cube_position_world.point.y}, {cube_position_world.point.z})")
                    print("-------------")
                    
                    # Create a Cube message - Cube is a custom message defined in ur5_control.msg
                    cube_msg = Cube()
                    cube_msg.position = cube_position_world.point
                    cube_msg.color = color_name

                    # Append cube data to array message
                    cube_array_msg.cubes.append(cube_msg)
                    
        # Publish the detected cubes
        cube_pub.publish(cube_array_msg)
        
        # Display the image with detected cubes
        cv2.imshow("Detected Cubes", cv_image)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr(e)

""" Function to transform pixel coordinates to world coordinates """
def transform_pixel_to_world(pixel_x, pixel_y):
    try:
        # We get the camera intrinsics from the K matrix:
        fx = 528.433756558705  # Focal length in x (pixels)
        fy = 528.433756558705  # Focal length in y (pixels)
        cx = 320.5             # Principal point x (pixels)
        cy = 240.5             # Principal point y (pixels)
        
        depth = 1.5 - 0.775  # Depth = Camera height - Table height (since object is at z=0.775 in world frame)

        X = depth                         # Depth now aligns with camera's Z-axis
        Y = -(pixel_x - cx) * depth / fx  # X in image corresponds to -Y in cam (moving horizontally in image, refers to moving in -Y-axis in real world coordinates w.r.t to camera, more info: on gitHub)
        Z = -(pixel_y - cy) * depth / fy  # Y in image corresponds to -Z in cam

        print("World coordinates: X_cam =", X, "Y_cam =", Y, "Z_cam =", Z)
        
        # a PointStamped message for the 3D camera coordinates
        point_camera = PointStamped()
        point_camera.header.frame_id = "camera_link"
        point_camera.header.stamp = rospy.Time(0)  # Use the latest available transform
        point_camera.point.x = X
        point_camera.point.y = Y
        point_camera.point.z = Z

        # Transform the point from the camera frame to the world frame
        transform = tf_buffer.lookup_transform("base_link", "camera_link", rospy.Time(0), rospy.Duration(1.0))
        point_world = tf2_geometry_msgs.do_transform_point(point_camera, transform)

        return point_world

    except Exception as e:
        rospy.logerr(f"Failed to transform pixel to world coordinates: {e}")
        return None

if __name__ == '__main__':
    rospy.init_node('color_cube_detector')

    # Initializing TF buffer and listener
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)
    
    # Initializing publisher to publish the cube position
    cube_pub = rospy.Publisher('/cube_positions', CubeArray, queue_size=10)

    # Subscribing to the camera image topic
    rospy.Subscriber('camera_top/image_raw', Image, image_callback)

    rospy.spin()