#!/usr/bin/env python3

""" Code to Spawn Objects in Gazebo"""

import rospy
import os
import subprocess
import time

def spawn_model(model_name, instance_name, x, y, z):
    model_path = os.path.expanduser(f"/home/esirem/Desktop/M2_Thesis/3-ROS/2-Simulations/UR5_Object_Sorting/ur5_color_sorting_ws/src/ur5_control/models/{model_name}/model.sdf")
    
    if not os.path.exists(model_path):
        rospy.logerr(f"Model {model_name} not found in ur5_control/models directory")
        return
    
    # running a terminal command from inside python: rosrun gazebo_ros spawn_model -file /path/to/my_cube.sdf -sdf -model red_cube_1 -x position_x -y position_y -z position_z

    cmd = [
        "rosrun", "gazebo_ros", "spawn_model",
        "-file", model_path,
        "-sdf",
        "-model", instance_name,  # Unique instance name
        "-x", str(x), "-y", str(y), "-z", str(z)
    ]
    subprocess.call(cmd)

if __name__ == "__main__":
    rospy.init_node("spawn_objects")

    time.sleep(2)

    rospy.loginfo("Spawning objects in Gazebo...")

    # Table (height = 0.75 meters)
    table_height = 0.75
    spawn_model("cafe_table", "table1", 0, 0.8, 0)

    # Cans
    can_height = 0.1
    # spawn_model("coke_can", "coca_cola", -0.3, 0.8, table_height + can_height / 2)
    # spawn_model("can_pepsi", "can_pepsi1", -0.3, 1, table_height + can_height / 2)
    # spawn_model("can_sprite", "can_sprite", 0.3, 0.8, table_height + can_height / 2)
    # spawn_model("can_fanta", "can_fanta", 0.3, 1, table_height + can_height / 2)

    # Cubes
    cube_height = 0.05
    spawn_model("red_cube", "cube_red", -0.2, 0.45, table_height + cube_height / 2)
    spawn_model("blue_cube", "cube_blue", 0.2, 0.45, table_height + cube_height / 2)
    spawn_model("green_cube", "cube_green", 0, 0.45, table_height + cube_height / 2)
    
    spawn_model("red_cube", "cube_red1", 0, 0.6, table_height + cube_height / 2)
    spawn_model("blue_cube", "cube_blue1", -0.2, 0.6, table_height + cube_height / 2)
    spawn_model("green_cube", "cube_green1", 0.2, 0.6, table_height + cube_height / 2)
    
    # # Bins
    spawn_model("red_bin", "red_bin1", 0.5, -0.55, 0) 
    spawn_model("blue_bin", "blue_bin1", -0.5, -0.55, 0)  
    spawn_model("green_bin", "green_bin1", 0, -0.55, 0) 
    
    rospy.loginfo("Objects spawned successfully!")