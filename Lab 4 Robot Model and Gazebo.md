# Lab 4: Robot Model and Gazebo

Seneca Polytechnic
AIG240 Robotics

## Introduction

### URDF

URDF (Unified Robot Description Format) is an XML format for representing a robot model. URDF is commonly used in Robot Operating System (ROS) tools such as RViz (ROS Visualization tool) and Gazebo simulator. It is essentially a 3D model with information about joints, motors, mass, etc. The files are then run through the Robot Operating System (ROS). The data from the file informs the human operator what the robot looks like and is capable of before they begin operating the robot.

More details on the URDF specification can be found [here](http://wiki.ros.org/urdf/XML).

Many robotic manufacturers have URDF models of their devices available for download. These include the [Segway RMP](https://stanleyinnovation.com/rmp-urdf-3d-files/), [TurtleBot](http://wiki.ros.org/turtlebot_description), and [AR10](https://github.com/Active8Robots/AR10/blob/master/ar10_description/urdf/ar10.urdf).

More Models:

- [Agility Robotics Digit](https://github.com/adubredu/DigitRobot.jl)
- [ANYbotics Anymal](https://github.com/ANYbotics/anymal_b_simple_description)
- [Boston Dynamics Spot (via Clearpath ROS Driver)](https://github.com/clearpathrobotics/spot_ros)
- [Clearpath Jackal](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/description.html)
- [Clearpath Dingo](https://www.clearpathrobotics.com/assets/guides/melodic/dingo/description.html)
- [Clearpath Husky](https://github.com/husky/husky)
- [Clearpath TurtleBot](http://wiki.ros.org/turtlebot_description)
- [Unitree Go1](https://github.com/unitreerobotics/unitree_ros)
- [Universal Robots](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)

### URDF vs XACRO

URDF (Unified Robot Description Format) and XACRO (XML Macros) are both used in ROS (Robot Operating System) for robot modeling, but they have distinct roles. URDF is a straightforward XML format that describes the physical structure of a robot, detailing its links and joints, as well as their properties like geometry and dynamics. It is ideal for simpler robot designs where the model's complexity is limited. In contrast, XACRO is an extension of URDF that incorporates macros, allowing for parameterization and reducing redundancy in model descriptions. This makes XACRO particularly useful for complex robots or those with interchangeable components, as it facilitates easier maintenance and enhances readability. Typically, XACRO files are processed into URDF files before they are utilized in simulations or applications, combining the flexibility of XACRO with the straightforwardness of URDF.

### RViz

RViz, or Robot Visualization, is a powerful 3D visualization tool used primarily in robotics and the Robot Operating System (ROS). It enables developers to visualize and interpret a wide array of sensor data, such as point clouds, maps, and robot models, in real-time. With its interactive features, users can manipulate objects and adjust visual settings to enhance understanding of robot behavior and performance. RViz's plugin architecture allows for extensibility, accommodating various data types and visualization needs. This makes it an invaluable resource for debugging algorithms, simulating scenarios, and gaining insights into robotic systems, ultimately aiding in the development and refinement of robotics applications.

### Gazebo

Gazebo is an open-source robotics simulation tool that provides a highly realistic environment for testing and developing robotic systems. It allows users to simulate robots in complex 3D environments, complete with detailed physics interactions, which include gravity, collisions, and friction. Gazebo supports a variety of sensors, such as cameras and LIDAR, enabling the generation of realistic sensor data for developing perception algorithms. Its seamless integration with the Robot Operating System (ROS) enhances its functionality, allowing developers to leverage ROS tools and libraries for robot control and communication. With a flexible plugin architecture, Gazebo can be customized to meet specific simulation needs, making it an essential platform for researchers and engineers in the field of robotics.

## Preparation

### JetAuto Robot

In preparation for using the JetAuto robot, please be familiar with the user manual and the basic lesson provided by the manufacturer found here:

- [JetAuto User Manual](https://seneca-bsa.github.io/bsa/aig240/JetAuto-User-Manual.pdf)
- [JetAuto & JetAuto Pro Resources](https://drive.google.com/drive/folders/16pwHYO8rK-22oAzStc7-olP9Weq7AbzY)

## Procedures

### Install Gazebo and other ROS packages

After becoming familiar with ROS, we'll now install the Gazebo simulation environment.

1. Install Gazebo version 9.X to be used with ROS Melodic. Each Gazebo version works with a specific version of ROS. More details about the installation can be found [here](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install).

   ```undefined
   sudo apt install gazebo9 libgazebo9-dev
   ```

2. Once installed, start Gazebo with the following command to ensure it is functional:

   ```undefined
   gazebo
   ```

   ![Figure 4.1 Gazebo Running](https://seneca-bsa.github.io/bsa/aig240/lab4-gazebo.png)

   ***Figure 4.1** Gazebo Running*

3. Lastly, ensure the following ROS Gazebo packages are installed:

   ```perl
   sudo apt install ros-melodic-gazebo-dev ros-melodic-gazebo-msgs ros-melodic-gazebo-plugins ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control ros-melodic-gazebo-ros-pkgs ros-melodic-joint-state-publisher ros-melodic-joint-state-publisher-gui ros-melodic-joint-trajectory-controller ros-melodic-moveit ros-melodic-trac-ik-kinematics-plugin ros-melodic-slam-gmapping
   ```

### Download the JetAuto Workspace to Local Machine for Simulation

1. For this lab, we'll use the JetAuto workspace `jetauto_ws` on our local virtual machine so we can test the robot locally in a simulation before programming the actual robot.

   [Download jetauto_ws.zip](https://drive.google.com/file/d/1SSaqoji_3H5vm9ZKAljeWPFmDUGfCc_u/view?usp=drive_link) and unzip it to the `jetauto` user's directory: `/home/jetauto/`

   If you downloaded the file on your host machine instead of your virtual machine, you may use WinSCP to transfer the file over to your virtual machine.

   **On your virtual machine:** Open a terminal and ensure `openssh-server` is installed.

   ```csharp
   sudo apt-get install openssh-server
   ```

   Enable and start SSH as necessary:

   ```bash
   sudo systemctl enable ssh
   sudo systemctl start ssh
   ```

   **On your host machine:** Open WinSCP and connect to: 127.0.0.1 port 22.

   Copy the `jetauto_ws` folder from `jetauto_ws.zip` into `/home/jetauto/`.

2. Once `jetauto_ws` is on your virtual machine's `jetauto` user directory, let's add it as a source in `~/.bashrc`.

   First, we need to change the workspace setup script to be an executable:

   ```bash
   sudo chmod +x /home/jetauto/jetauto_ws/devel/_setup_util.py
   ```

   Then:

   ```bash
   echo "source /home/jetauto/jetauto_ws/devel/setup.bash" >> ~/.bashrc
   ```

   Use `nano` to check if the `source` line got added to the end of `~/.bashrc`

   ```bash
   nano ~/.bashrc
   ```

### JetAuto Robot Model

Now that the JetAuto robot workspace is on the virtual machine, let's try to simulate it in Gazebo. A robot model in URDF consists of links that are joined together to form a robot assembly. Each link has its given geometry, mass, and collision parameters. The geometry can be provided as a simple shape or a complex shape using a solid model.

1. Before we start, let's ensure (double-check) we have the required packages installed (if you haven't installed them from the beginning of this lab) to view and test our robot model:

   ```perl
   sudo apt install ros-melodic-joint-state-publisher ros-melodic-joint-state-publisher-gui ros-melodic-joint-trajectory-controller
   ```

2. Next, we need to set some environment variables that our launch script will look for. Edit the system's environment variables:

   ```bash
   sudo gedit /etc/environment
   ```

   We'll add the following in the environment:

   ```ini
   LIDAR_TYPE="A1"
   DEPTH_CAMERA_TYPE="AstraProPlus"
   MACHINE_TYPE="JetAutoPro"
   HOST="/"
   MASTER="/"
   ```

   **Restart your virtual machine in order for the changes to take effect.**

3. Now, let's have a quick view of the URDF model. We can use RViz for visualization. A launch file allows us to start multiple nodes at once as well as define other attributes.

   ```bash
   roslaunch jetauto_description display.launch model:=urdf/jetauto.urdf
   ```

   You may inspect this particular launch file at the following location:

   **~/jetauto_ws/src/jetauto_simulations/jetauto_description/launch/display.launch**

   In it, you'll find it starts three nodes:

   - joint_state_publisher_gui
   - robot_state_publisher
   - rviz

   You can find more information about `roslaunch` and `.launch` files in the official ROS tutorials: [roslaunch](https://wiki.ros.org/roslaunch) and [Roslaunch tips for large projects](https://wiki.ros.org/ROS/Tutorials/Roslaunch tips for larger projects).

4. Once RViz is started, you can use the `joint_state_publisher_gui` to adjust the arm angle.

   ![Figure 4.3 JetAuto in RViz](https://seneca-bsa.github.io/bsa/aig240/lab4-rviz-jetauto.png)

   ***Figure 4.3** JetAuto in RViz* *(Note: The original image caption said "JetAuto in Gazebo", but the context and image refer to RViz)*

5. Let's open up the JetAuto URDF model file to take a closer look at it.

   **~/jetauto_ws/src/jetauto_simulations/jetauto_description/urdf/jetauto_car.urdf.xacro**

   Here, we see a file in XML format:

   ```xml
   <robot name="jetauto" xmlns:xacro="http://ros.org/wiki/xacro" >
       <xacro:property name="M_PI"               value="3.1415926535897931"/>
       <xacro:property name="base_link_mass"     value="1.6" /> 
       <xacro:property name="base_link_w"        value="0.297"/>
       <xacro:property name="base_link_h"        value="0.145"/>
       <xacro:property name="base_link_d"        value="0.11255"/>
   
       <xacro:property name="wheel_link_mass"    value="0.1" />
       <xacro:property name="wheel_link_radius"  value="0.049"/>
       <xacro:property name="wheel_link_length"  value="0.04167"/>
   ```

   The first few lines define the robot's name and the basic parameters of the JetAuto's body.

   - `M_PI` defines the value of π.
   - `base_link_mass` defines the mass of the JetAuto’s body model.
   - `base_link_w` defines the width of the JetAuto’s body model.
   - `base_link_h` defines the height of the JetAuto’s body model.
   - `base_link_d` defines the length of the JetAuto’s body model.
   - `wheel_link_mass` defines the mass of each Mecanum wheel.
   - `wheel_link_radius` defines the radius of each Mecanum wheel.

   The name of the robot is also defined as `jetauto`.

   ```xml
   <link name="base_footprint"/>
   
   <joint name="base_joint" type="fixed">
       <parent link="base_footprint"/>
       <child link="base_link"/>
       <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
   </joint>
   ```

   `base_footprint` is defined as the top parent link (part) of the JetAuto model to create an overall envelope that sits at the origin. `base_link` is the base part of the robot that houses the battery and motor. In the URDF model, it is connected to `base_footprint` as a child link. This envelope configuration ensures the wheels of the robot will always be above the origin (ground).

   ```xml
   <link
       name="base_link">
       <xacro:box_inertial m="${base_link_mass}" w="${base_link_w}" h="${base_link_h}" d="${base_link_d}"/>
       <visual>
           <origin
               xyz="0 0 0"
               rpy="0 0 0" />
           <geometry>
               <mesh
                   filename="package://jetauto_description/meshes/base_link.stl" />
           </geometry>
           <material name="green"/>
       </visual>
       <collision>
           <origin
               xyz="${base_link_w/2.0 - 0.14810} 0 ${0.126437/2 + 0.02362364}"
               rpy="0 0 0" />
           <geometry>
               <box size="${base_link_w} ${base_link_h} ${base_link_d}" />
           </geometry>
       </collision>
   </link>
   ```

   Next is the link/part `base_link` along with its elements. The mass and inertial information of the part is defined as an XACRO element. The `geometry` sub-element in the `visual` element is provided by an `stl` mesh file from the `jetauto_description` package. The `collision` element is also defined as a box relative to the specified XYZ coordinate.

   ![Figure 4.4 JetAuto base_link STL](https://seneca-bsa.github.io/bsa/aig240/lab4-base_link-stl.png)

   ***Figure 4.4** JetAuto base_link STL*

   ```xml
   <link
       name="back_shell_link">
       <inertial>
       <origin
           xyz="-1.22838595456587E-05 0.00218574826309681 -0.0500522861933898"
           rpy="0 0 0" />
       <mass
           value="0.0663478534899862" />
       <inertia
           ixx="5.65277934912267E-05"
           ixy="-5.13394387877366E-11"
           ixz="-4.07561372273553E-11"
           iyy="4.33740893441632E-05"
           iyz="-5.43059341238134E-06"
           izz="6.86642544694324E-05" />
       </inertial>
       <visual>
           <origin
               xyz="0 0 0"
               rpy="0 0 0" />
           <geometry>
               <mesh
               filename="package://jetauto_description/meshes/back_shell_link.stl" />
           </geometry>
           <material name="black">
           </material>
       </visual>
       <collision>
           <origin
               xyz="0 0 0"
               rpy="0 0 0" />
           <geometry>
               <mesh
               filename="package://jetauto_description/meshes/back_shell_link.stl" />
           </geometry>
       </collision>
   </link>
   <joint
       name="back_shell_joint"
       type="fixed">
       <origin
           xyz="-0.076481 0 0.082796"
           rpy="-3.1416 0 1.5708" />
       <parent
           link="base_link" />
       <child
           link="back_shell_link" />
       <axis
           xyz="0 0 0" />
   </joint>
   ```

   The `back_shell_link` is the part that houses the Jetson Nano, the expansion board, and mounts the antenna. All the elements are defined in a similar manner as `base_link`, and it is defined as a child link of `base_link` with its relative position defined in `joint`.

   The `wheel_XXX_link` are all defined in a similar manner.

6. The JetAuto URDF model file above only defines the mechanical structure of the robot. If we take a look at the URDF file for simulating the robot in Gazebo, we will find more links that are used and defined in other URDF files within the same package.

   **~/jetauto_ws/src/jetauto_simulations/jetauto_description/urdf/jetauto.xacro**

7. If you are interested in building a URDF from scratch, visit the ROS tutorial [here](https://wiki.ros.org/urdf/Tutorials).

8. Close all terminals.

### Running JetAuto robot in Gazebo

1. In a terminal, launch:

   ```undefined
   roslaunch jetauto_gazebo worlds.launch
   ```

   Gazebo should run, and you should see the JetAuto robot in the simulation environment.

   ![Figure 4.5 JetAuto in Gazebo](https://seneca-bsa.github.io/bsa/aig240/lab4-gazebo-jetauto.png)

   ***Figure 4.5** JetAuto in Gazebo*

2. With Gazebo and ROS running, we can now control the virtual robot the same way as the physical robot.

   Let's try publishing to the `cmd_vel` topic. Open a new terminal and run:

   ```bash
   rostopic pub -1 /jetauto_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
   ```

   Stop the robot:

   ```bash
   rostopic pub -1 /jetauto_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
   ```

   Do you notice the JetAuto robot listens to the same `cmd_vel` topics and uses the same `Twist` message as `TurtleSim`?

3. Let's inspect the source code of the motion controller to take a closer look. Open the file at the following path:

   **~/jetauto_ws/src/jetauto_driver/jetauto_controller/scripts/jetauto_controller_main.py**

   As we can see in the controller code:

   ```lua
   linear_x = self.go_factor*msg.linear.x
   linear_y = self.go_factor*msg.linear.y
   angular_z = self.turn_factor*msg.angular.z
   
   speed_up = False
   if abs(self.last_linear_x - linear_x) > 0.2 or abs(self.last_linear_y - linear_y) > 0.2 or abs(self.last_angular_z - angular_z) > 1:
       speed_up = True
   
   self.last_linear_x = linear_x
   self.last_linear_y = linear_y
   self.last_angular_z = angular_z
   
   linear_x_, linear_y_ = linear_x * 1000.0, linear_y * 1000.0 #mm to m
   speed = math.sqrt(linear_x_ ** 2 + linear_y_ ** 2)
   direction =  math.atan2(linear_y_, linear_x_)
   direction = math.pi * 2 + direction if direction < 0 else direction
   self.mecanum.set_velocity(speed, direction, angular_z, speed_up=speed_up)
   ```

   The `Twist` message from `cmd_vel` provides the necessary information to calculate `speed`, `direction`, and `angular_z` for controlling the Mecanum wheels using the `MecanumChassis` object.

   Refer to [JetAuto & JetAuto Pro Resources](https://drive.google.com/drive/folders/16pwHYO8rK-22oAzStc7-olP9Weq7AbzY) chapter 7.3 for the working principle of the Mecanum wheel.

4. Try other various combinations of motion commands to gain a better understanding of the robot's movement, such as changing the linear values for both `x` and `y` as well as the angular values.

5. Next, we'll try controlling the JetAuto robot using keyboard input. In a new/other terminal, run:

   ```go
   roslaunch jetauto_peripherals teleop_key_control.launch robot_name:="/"
   ```

   Use w, a, s, d to control the robot.

   **Troubleshooting:** If you get a `No module named 'rospkg'` error, it means there are some errors with the python interpretor and the ROS package. First, ensure `rospkg` is installed:

   ```undefined
   sudo apt install python-rospkg
   ```

   **Troubleshooting:** If the problem presist, try removing `python-rospkg` which will also remove a long list of `ros` modules. Then re-install ROS.

   ```csharp
   sudo apt remove python-rospkg
   sudo apt install ros-melodic-desktop-full
   ```

   **Make sure you re-install all the dependency at the start of this lab as well.**

6. Inspect the source code of the teleop controller to understand its operation by opening the file at:

   **~/jetauto_ws/src/jetauto_peripherals/scripts/teleop_key_control.py**

## Lab Question

1. Modify the controller you created from [lab3](https://seneca-bsa.github.io/bsa/aig240/lab3/) (or create a new one) so it will publish to the `/jetauto_controller/cmd_vel` topic for controlling the JetAuto robot in Gazebo.

   **Hint:** You can follow the same approach as Lab 3 by creating a new package called `lab4_jetauto_control` in your `ros_ws`.

   ```undefined
   catkin_create_pkg lab4_jetauto_control rospy geometry_msgs
   ```

   Refer to the `teleop_key_control.py` controller you used in this lab on how to publish to the JetAuto nodes.