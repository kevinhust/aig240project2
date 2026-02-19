# Lab 5 : JetAuto Robot

Seneca Polytechnic
AIG240 Robotics

### JetAuto Robot Inspection

The robot we are using for this course is the JetAuto Pro assembled in the configuration:

![Figure 5.2 JetAuto Pro](https://seneca-bsa.github.io/bsa/aig240/lab5-jetauto-pro.png)

***Figure 4.2** JetAuto Pro*

1. Before using the JetAuto robot, read the following:

   - [JetAuto User Manual](https://seneca-bsa.github.io/bsa/aig240/JetAuto-User-Manual.pdf)
     - Page 01: Guide to Battery Safety
     - Page 03: JetAuto Pro Standard Kit Packing List
     - Page 04-09: Installation Instruction (except for 1.4 LCD)
       - Check all nuts and bolts to ensure confirm installation and security
     - Page 10-11: Charging and Starting the Robot

   We will NOT be using the smartphone app for controlling the robot.

2. Go through "1.2 Install 3D Depth Camera" on Page 4 to "2. Start the Robot" on Page 10-11. We will NOT be using the smartphone app for controlling the robot.

   **NOTE:** All the cables, nuts, and bolts are already installed. You are to validate that they are not missing or loose.

### SSH into the JetAuto Robot

**Copy this lab instruction somewhere on your computer as you'll lose connection to the internet!**

1. By default, the JetAuto is configured to be in Wifi AP mode. Power on the robot and connect to the robot's WiFi starting in "HW-". If you are unsure of which Wifi SSID is your robot broadcasting, open the "Tool" application on the robot and look for the AP name in the setting. Do NOT change any of the default settings.

   The password for the WiFi connection is: **hiwonder**

   Remember, the Wifi AP from the JetAuto have no access to the internet so it's alright to see "No Internet" or similar warning when connecting to it.

2. If your robot is making a high pitch beeping sound, that means the battery voltage is low. Plugging the robot into it's charger should solve the problem but if the battery haven't been charged for a while, you might need to leave the robot off and charge it for 10-15 minutes before powering it on.

3. Once connected, use terminal (or PuTTY) to SSH into the robot at "192.168.149.1".

   ```css
   ssh jetauto@192.168.149.1
   ```

   The user is: **jetauto**, and the password is: **hiwonder**

   #### Option 2: USB connection with the robot

   1. It is also possible to connect with the robot via USB using the Jetson Nano's micro-B USB port if you do not want to loose internet connection. However, you'll only be aable to use the command line interface with this method.

   2. Use `screen` or similar serial terminal application to connection with the robot.

      ```bash
      sudo apt-get install -y screen
      sudo screen /dev/ttyACM0 115200
      ```

      **NOTE:** The port name may vary.

### Remote Desktop into the JetAuto Robot

1. **Only recommended to be used on within your virtual machine.** For security, do not install NoMachine on your host computer and stop the NoMachine server after your installation.

   NoMachine is a tool that allows for remote desktop and access. It is a powerful tool that is pre-installed on the JetAuto image but use with caution as it will also automatically create a start a remote access server after installaion open up your machine for remote connection.

   Disconnect your computer from the JetAuto robot and re-connect to the internet to [Download and Install NoMachine](https://downloads.nomachine.com/everybody/).

2. After installing NoMachine, reconnect to JetAuto's Wifi AP and you should be able to search for the JetAuto robot from NoMachine.

   Once you are connected with the robot, the credential is the same as above.

3. Now that you have two methods in connecting with the JetAuto robot, you may use either one to control the robot. Keep in mind, the SSH method is running off your virtual machine and connecting to JetAuto using command line where the NoMachine is connecting to JetAuto robot's actual desktop (remote desktop). GUI tools such as RViz will not work on command line interface.

4. To conserve battery life, once you've connected to your robot, you may unplug the LCD screen but remember that the LCD screen must be plugged in during power on in order for the robot to detect it for displaying the desktop.

### JetAuto Robot Movement

- Ensure the battery charging cable is UNPLUGGED and all cables on the robot are secure.
- Ensure all structures, nuts and bolts on the robot are tightly fastened.
- Ensure the robot is on the ground and awy from any obstacles.

1. In a terminal that's connected to the JetAuto robot using SSH or in JetAuto's terminal using remote desktop, stop the app service then start the `jetauto_controller` service:

   You must stop this service everytime you want to control the robot using your own script through ROS because the JetAuto robot automatically start an App service to allow for control using an Android or iOS application.

   ```vbnet
   sudo systemctl stop start_app_node.service
   ```

   Launch the robot controller for controlling the robot's hardware.

   ```undefined
   roslaunch jetauto_controller jetauto_controller.launch
   ```

2. Now that the controller service has started, we can publish move command as Twist message to the motion controller (similar to what we did in lab 4). Before publishing a command to the robot, remember you must issue a stop command for the robot to stop. Let's issue the stop command first so you can recall it faster.

   ```bash
   rostopic pub -1 /jetauto_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
   ```

   Your robot should not do anything.

3. Place the robot on the ground and be ready to issue the stop command immediately after. Issue a move command to the robot in the x-direction (forward) at 0.3 m/s, `x: 0.3`:

   ```bash
   rostopic pub -1 /jetauto_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
   ```

   Your robot should now start moving. Be ready to stop the robot by issuing (or up arrow twice):

   ```bash
   rostopic pub -1 /jetauto_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
   ```

   

   - DO NOT set the movement value above 0.7 m/s to keep the robot within its control limit.

   

   The linear values refer to the translation of the robot. Positive X is forward and positive Y is left. There is no Z-direction for this robot. Do not exceed 0.7 m/s.

   The angular values refer to the rotation of the robot. Only Z-rotation is considered with positive value as counter-clockwise. Do not exceed 3.5 rad/s.

4. Next, try using the controller you created in Lab 4 to control the robot by copying you Lab 4 workspace over to the home directory on the JetAuto robot. You do not need to copy the `jetauto_ws` workspace as it's already on the JetAuto robot. You may use ssh, sftp, winscp, or any file transfer method.

5. Once your new workspace is on the JetAuto robot, source it the same way as how you've done it in Lab 3 and Lab 4. Afterward, you'll be able to run the script just like how you've run it on Gazebo.

## Lab Question

1. Using the code from Project 2, modify it so the JetAuto robot will move in a real-world roughly 1-meter square shape pattern as shown below. Remember, you'll be implementing it on the physical JetAuto robot so the dyanmics will not be the same. ie. You robot will most likely under or overshoot the movement.

![Figure 1 Square Movement Pattern](https://seneca-bsa.github.io/bsa/aig240/lab4-task.png)

***Figure 1** Square Movement Pattern*

Start:

1. Move forward from (0, 0, 0°) to (1, 0, 0°) facing the direction of travel; then
2. Move left sideway from (1, 0, 0°) to (1, 1, 0°) without turning, so the robot is facing the outside of the square; then
3. Turn clockwise from (1, 1, 0°) to (1, 1, -90°) to face into the square; then
4. Move right sideway from (1, 1, -90°) to (0, 1, -90°) facing the inside of the square; then
5. Move forward and turn from (0, 1, -90°) to (0, 0, 0°) by rotating the robot while traveling.

Repeat this 2 times after a start command (such as a keyboard input) is given.

For example: after launching the JetAuto in Gazebo, when you run `rosrun lab5_jetauto_control jetauto_control`, it will ask for input before performing the above action.

**Hint:** You can follow the same approach as Lab 3 by creating a new package called `lab5_jetauto_control` in your `ros_ws`.

```markdown
    catkin_create_pkg lab5_jetauto_control rospy geometry_msgs
```

## 