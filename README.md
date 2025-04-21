# UR5 Vision based Pick and Place using MoveIt and ROS
This repository contains a ROS-based solution for vision-guided pick-and-place tasks using a Universal Robots UR5 robotic arm equipped with a Robotiq 85 2-finger gripper. The project integrates computer vision with robotic manipulation to enable colour based object sorting.

The main contribuation of this repo is a custom package named **"UR5_control"** along with a MoveIt configuration package called **"ur5_gripper_moveit_config"**. 

👉 Gazebo is being used for simulation of the arm. 🤖🛠

##### 📺 Demo Video
You can watch the demo video of the arm in action by clicking on the below image:
[![Watch the video](https://github.com/EhtishamAshraf/Robotic_Arm_Vision_Pick_Place/blob/33d23d7b45f0b756a5b1dc75e1e1707daca08657/ur5_control/Images/gazebo.png)](https://youtu.be/jUzRy0JSo-M)

## 📥 Cloning the Repository
Create a ros workspace, inside it, create a src folder and navigate into it and run the following command to clone the repo:
```bash
git clone https://github.com/EhtishamAshraf/Robotic_Arm_Vision_Pick_Place.git
```
Run "catkin_make" inside the main workspace, to build the workspace and compile all ROS packages, ensuring that your system recognizes the newly cloned code.
```bash
catkin_make 
```

# 🌐 Simulation

🎯 **Random object position:**

The object can be placed anywhere within the camera's field of view. The algorithm automatically detects the object, identifies its color, determines its location in real-world coordinates, and proceeds to pick and sort it based on color classification.
![Demo in Action](https://github.com/EhtishamAshraf/Robotic_Arm_Vision_Pick_Place/blob/2e53db006fcf785844715a155fc84e6350c14991/ur5_control/Images/Video2_Cube%20random%20Position.gif)

🤖⚙️ **UR5 Robotic Arm**

In robotics , **Forward Kinematics (FK)** and **Inverse Kinematics (IK)** are essential techniques used to describe and control the motion of robotic arms.

In ROS, **Controllers** are used in simulation to command and control the virtual robot’s joints and movements accurately and realistically.

👉 [`Inverse Kinematics Repo`](https://github.com/EhtishamAshraf/ROS_Robotic_Arm_IK.git) – refer for more information on the Kinematics of UR5 and ROS controllers used in Simulation.

📷 **Camera Role and Coordinate Transformation**

The **camera** plays a critical role in this project by capturing live video feed. When objects (in current setup; cubes) appear within the camera's field of view (FOV), the system:

1. Captures images of the cubes.
2. Processes the images to detect the color of each cube.
3. Identifies the pixel coordinates of the detected cubes.
4. Converts the pixel coordinates into camera frame coordinates.
5. Transforms the camera frame coordinates into real-world coordinates for precise robotic manipulation.

The transformation from **pixel coordinates** to **camera coordinates** is performed using the following equations:

X = depth

Y = -((pixel_x - cx) * depth) / fx

Z = -((pixel_y - cy) * depth) / fy

> **Note**:  
> - As the camera is mounted facing downward, the frame axes are inverted.
> - This inversion causes the X-axis to align directly with the **depth** value (distance from the camera to the object).
> - In the image plane:
> - Pixel values increase **horizontally** (left to right) along the X-axis.
> - Pixel values increase **vertically downward** along the Y-axis.
> - However, **with the current setup**, the **camera's Y-axis** aligns with the image's **horizontal X-direction**, and the **camera's Z-axis** aligns with the image's **vertical downward Y-direction**.
> - Consequently, the Y and Z coordinates are negative relative to the image axes.  

The below image shows the frame of the camera in rviz:
![camera frame](https://github.com/EhtishamAshraf/Robotic_Arm_Vision_Pick_Place/blob/2e53db006fcf785844715a155fc84e6350c14991/ur5_control/Images/camera_frame.png)

The camera captures the scene and the detection algorithm identifies individual cubes within the field of view. For each detected cube, the algorithm computes the centroid, draws a bounding box around it, and displays the identified color label. This provides both visual feedback and critical information for further processing steps such as coordinate transformation and pick-and-place operations.

![detected cubes](https://github.com/EhtishamAshraf/Robotic_Arm_Vision_Pick_Place/blob/2e53db006fcf785844715a155fc84e6350c14991/ur5_control/Images/detected_cubes.png)

👉 [`Color based detection`](https://github.com/EhtishamAshraf/Turtlebot3_line_wall_following.git) – refer for more information on color based detection using Python. {Section: Line Following Logic}

In the images below, the UR5 robotic arm is shown sequentially picking and placing red cubes. The arm processes and sorts cubes based on a color-priority order: red → green → blue. Each cube is identified by color, picked individually, and placed at the designated location.

Processing first red cube:
![red 1](https://github.com/EhtishamAshraf/Robotic_Arm_Vision_Pick_Place/blob/2bdcb93888f11f9191fcc4407279d0fa3aa5576b/ur5_control/Images/red_cube_1_picked.png)
Processing second red cube:
![red 2](https://github.com/EhtishamAshraf/Robotic_Arm_Vision_Pick_Place/blob/2bdcb93888f11f9191fcc4407279d0fa3aa5576b/ur5_control/Images/red_cube_2_picked.png)

📷🔧 **Camera calibration parameters:**

To retrieve the camera calibration parameters, use the following command in the ROS environment: `rostopic echo /camera_top/camera_info`

This will output the camera calibration information, which includes intrinsic and extrinsic parameters necessary for accurate image processing and camera modeling.

The example below demonstrates the expected output format of this command:
![camera_info](https://github.com/EhtishamAshraf/Robotic_Arm_Vision_Pick_Place/blob/2e53db006fcf785844715a155fc84e6350c14991/ur5_control/Images/camera_info.png)

🛠️🚧 **Obstacles in MoveIt:**
In the current setup, the table and bins are added as obstacles within the MoveIt environment to ensure that the arm's motion planning avoids accidental collisions. By incorporating these obstacles into the planning scene, the robot can intelligently navigate its workspace and plan its movements while accounting for surrounding objects, preventing potential collisions and ensuring smooth and safe operation.

This setup enhances the accuracy and reliability of the arm's path planning, making it more efficient for real-world tasks.

![detected cubes](https://github.com/EhtishamAshraf/Robotic_Arm_Vision_Pick_Place/blob/2e53db006fcf785844715a155fc84e6350c14991/ur5_control/Images/rviz.png)


🤖🪝 **Handling Object Gripping in Gazebo:**

In the Gazebo simulation, the cubes are affected by gravity. To prevent the objects from falling during the gripper's operations, gravity is temporarily disabled while picking up the objects and re-enabled when placing them. This ensures that the cubes remain securely in the gripper's hold during the operation.

Additionally, stiffness coefficients and friction parameters have been applied to the cubes. These adjustments help improve the gripper's ability to securely grip the cubes, simulating a more realistic interaction between the gripper and the objects.

👉 Please note that these modifications are specific to the simulation environment and do not affect real-world scenarios.

## ⚙️ Execution: Follow the steps to launch and run the repository. 🚀
⚠️ Note: The Gazebo simulation is intentionally launched in a paused state to allow all components and controllers to initialize properly before starting.

✅ Remember to manually unpause the simulation in the Gazebo GUI once everything is loaded.

- Open a new terminal on your laptop (navigate to ~/path_to_your_workspace/src), 
  and run the following command to initialize the Gazebo and Rviz with the UR5 arm:
```bash
roslaunch ur5_control spawn_ur5_gripper.launch
```
- run the node to spawn cubes, table, and bins in the simulation by typing the following command:
```bash
rosrun ur5_control spawn_objects.py
```
- run the cube detection node by typing the following command:
```bash
rosrun ur5_control cube_detector.py
```
- main pick N place node can be run by typing the following commandon the terminal:
```bash
rosrun ur5_control vision_pick_N_place.py
```
⚡ Make sure to run `source devel/setup.bash` in each new terminal before typing in the above commands.



