#!/usr/bin/env python3
"""
Code to sort red, green, blue cubes based on color:
->  Bins positions is fixed
->  Object's position could be anywhere within the camera's FOV.
->  If all the objects are sorted successfully the arm will go the UP position.
->  The arm sorts objects in a specific order Red -> Green -> Blue
->  The camera attached to the gripper works but hasn't been used for picking and placing. 
->  The camera present at the top of the workspace is used in current setup

Note:
Take robot to a specific position using moveit, once the robot reaches the position in rviz and gazebo, 
then run the following command to get the: 
Translation and rotation of gripper (position in: xyz, and rotation in rpy).
 
command $: rosrun tf tf_echo /base_link /tool0
"""

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations as tf
from geometry_msgs.msg import PoseStamped
from ur5_control.msg import CubeArray

height_offset = 0.68            # Compensation for gripper length (Because the motion planning is done without gripper, so if this offset is not added, the gripper will hit objects rather than picking them)
robot_busy = False              # Flag to track robot state
processed_cubes = set()         # Store already picked cube positions

"""Function: to close gripper in order to pick up the objects"""
def close_gripper(gripper):
    rospy.loginfo("Closing the gripper...")
    gripper.set_joint_value_target({"robotiq_85_left_knuckle_joint": 0.36})
    success = gripper.go(wait=True)
    gripper.stop()
    if success:
        rospy.loginfo("Object picked.")
    else:
        rospy.logerr("Failed to pick object.")
        
"""Function: to open gripper in order to place the objects"""
def open_gripper(gripper):
    rospy.loginfo("Opening the gripper...")
    gripper.set_joint_value_target({"robotiq_85_left_knuckle_joint": 0.0})
    success = gripper.go(wait=True)
    gripper.stop()
    if success:
        rospy.loginfo("Object placed.")
    else:
        rospy.logerr("Failed to place object.")

"""Function: to reach specific poses using MoveIt"""
def move_to_pose(group, target_pose):
    group.set_pose_target(target_pose, end_effector_link="tool0")   # set the target pose
    
    success, plan, _, _ = group.plan()                              # plan motion using MoveIt
    # checking if motion planning is successful:
    if success and len(plan.joint_trajectory.points) > 0:           # If a valid path is found, execute motion
        print("Valid plan found. Executing...")
        group.execute(plan, wait=True)
        group.go(wait=True)
    else:
        print("Planning failed. No valid motion plan found.")
        return False
    
    # stop once, execution is done:
    group.stop()
    group.clear_pose_targets()
    # rospy.sleep(1.0)
    return success

# Callback function for the topic /cube_positions, cube's real time positions are published on this topic
def cube_callback(msg):
    global robot_busy, processed_cubes
    
    if robot_busy:
        return                                  # Ignore new cubes while processing

    if not msg.cubes:
        rospy.loginfo("No cubes detected.")
        return

    # Find the first unprocessed cube
    for cube in msg.cubes:
        cube_position = (round(cube.position.x, 2), round(cube.position.y, 2), round(cube.position.z, 2))
        
        if cube_position not in processed_cubes:
            rospy.loginfo(f"Processing new cube: Color={cube.color}, Position={cube_position}")
            processed_cubes.add(cube_position)                  # Mark as processed once processing of a cube is started
            robot_busy = True
            pick_and_place(cube)                                # pick N place pipeline
            robot_busy = False
            return
    
    # Once all 6 cubes are processed, go to a specific Home position
    rospy.loginfo("All detected cubes have been processed. Waiting for new cubes...")
    
    rospy.loginfo(f"Moving to Up position: ")
    target_joints = [0, -1.5653, 0.0366, 0, 0, 0]
    group.set_joint_value_target(target_joints)
    success = group.go(wait=True)
    group.stop()
    if success:
        rospy.loginfo("Movement successful...")
    else:
        rospy.logwarn("Motion planning failed! Check joint limits or obstacles.")

"""Function: to pick and place the cube"""    
def pick_and_place(cube):
    global group, gripper
    
    fixed_orientation = tf.quaternion_from_euler(3.119, 0.010, -3.124)
    
    # Move above the cube
    pre_pick_pose = geometry_msgs.msg.Pose()
    pre_pick_pose.position.x = cube.position.x
    pre_pick_pose.position.y = cube.position.y
    pre_pick_pose.position.z = cube.position.z + height_offset + 0.25  # Hover above
    pre_pick_pose.orientation.x = fixed_orientation[0]
    pre_pick_pose.orientation.y = fixed_orientation[1]
    pre_pick_pose.orientation.z = fixed_orientation[2]
    pre_pick_pose.orientation.w = fixed_orientation[3]
    move_to_pose(group, pre_pick_pose)
    
    # Move to pick position
    pick_pose = geometry_msgs.msg.Pose()
    pick_pose.position.x = cube.position.x
    pick_pose.position.y = cube.position.y
    pick_pose.position.z = cube.position.z + height_offset
    pick_pose.orientation = pre_pick_pose.orientation
    move_to_pose(group, pick_pose)
    
    # Close gripper
    close_gripper(gripper)
    
    # Lift cube
    move_to_pose(group, pre_pick_pose)
    
    # Define drop positions for different colors
    drop_positions = {
        "red": [0.5, -0.5, 0.5],
        "green": [0.0, -0.5, 0.5],
        "blue": [-0.5, -0.5, 0.5]
    }
    
    color = str(cube.color)         # Ensuring color is a string
    drop_x, drop_y, drop_z = drop_positions.get(color, [0, 0, 1])  # [0,0,1] is Default position in case of unknown color
    
    # Move above drop position
    pre_drop_pose = geometry_msgs.msg.Pose()
    pre_drop_pose.position.x = drop_x
    pre_drop_pose.position.y = drop_y
    pre_drop_pose.position.z = drop_z + 0.15
    pre_drop_pose.orientation = pre_pick_pose.orientation
    move_to_pose(group, pre_drop_pose)
    
    # Move to drop position
    drop_pose = geometry_msgs.msg.Pose()
    drop_pose.position.x = drop_x
    drop_pose.position.y = drop_y
    drop_pose.position.z = drop_z
    drop_pose.orientation = pre_pick_pose.orientation
    move_to_pose(group, drop_pose)
    
    # Open gripper
    open_gripper(gripper)
    
    # Move back to pre-drop position
    move_to_pose(group, pre_drop_pose)

""" Main function to initialize the node and set up the scene """
def main():
    global group, gripper
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cube_picking_node', anonymous=True)
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")
    gripper = moveit_commander.MoveGroupCommander("gripper")
    
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)
    group.set_planning_time(10.0)
    
    rospy.sleep(1)
    
    # Add table as a collision object
    table_pose = PoseStamped()
    table_pose.header.frame_id = "world"  # Add obstacle w.r.t to the world frame
    table_pose.pose.position.x = 0
    table_pose.pose.position.y = 0.8
    table_pose.pose.position.z = 0.375
    scene.add_box("table", table_pose, size=(0.913, 0.913, 0.775))  # Adjusted table size
    rospy.sleep(0.5)
    
    # Add red bin as a collision object
    red_bin_pose = PoseStamped()
    red_bin_pose.header.frame_id = "world"  
    red_bin_pose.pose.position.x = 0.5
    red_bin_pose.pose.position.y = -0.5
    red_bin_pose.pose.position.z = 0.11
    scene.add_box("red_bin", red_bin_pose, size=(0.2, 0.2, 0.22))  
    rospy.sleep(0.5)
    
    # Add green bin as a collision object
    green_bin_pose = PoseStamped()
    green_bin_pose.header.frame_id = "world"  
    green_bin_pose.pose.position.x = 0.0
    green_bin_pose.pose.position.y = -0.5
    green_bin_pose.pose.position.z = 0.11
    scene.add_box("green_bin", green_bin_pose, size=(0.2, 0.2, 0.22))  
    rospy.sleep(0.5)
    
    # Add blue bin as a collision object
    blue_bin_pose = PoseStamped()
    blue_bin_pose.header.frame_id = "world"  
    blue_bin_pose.pose.position.x = -0.5
    blue_bin_pose.pose.position.y = -0.5
    blue_bin_pose.pose.position.z = 0.11
    scene.add_box("blue_bin", blue_bin_pose, size=(0.2, 0.2, 0.22))  
    rospy.sleep(0.5)
    
    rospy.Subscriber("/cube_positions", CubeArray, cube_callback) # subscribe to this topic to get cube positions
    rospy.loginfo("Waiting for cube positions...")
    rospy.spin()
    
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
