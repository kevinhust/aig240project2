# Project 2 - Complete Setup & Run Instructions

**AIG240 Robotics - Seneca Polytechnic**

---

## Table of Contents

1. [Overview](#overview)
2. [Setup on Linux VM](#setup-on-linux-vm)
3. [Running the Project](#running-the-project)
4. [Troubleshooting](#troubleshooting)
5. [Assessment Questions](#assessment-questions)
6. [Submission Checklist](#submission-checklist)

---

## Overview

This project controls the **JetAuto robot** in Gazebo to move in a 1-meter square pattern:

```
Square Pattern (repeated 2 times):

Step 1: Forward      (0, 0, 0°)   → (1, 0, 0°)
Step 2: Left strafe  (1, 0, 0°)   → (1, 1, 0°)
Step 3: Turn CW      (1, 1, 0°)   → (1, 1, -90°)
Step 4: Right strafe (1, 1, -90°) → (0, 1, -90°)
Step 5: Diagonal     (0, 1, -90°) → (0, 0, 0°)
```

**Topic:** `/jetauto_controller/cmd_vel`
**Message Type:** `geometry_msgs/Twist`

---

## Setup on Linux VM

### Step 1: Install Gazebo

```bash
sudo apt update
sudo apt install gazebo9 libgazebo9-dev
```

Verify installation:

```bash
gazebo
```

Close Gazebo after testing.

### Step 2: Install ROS Gazebo Packages

```bash
sudo apt install ros-melodic-gazebo-dev \
                 ros-melodic-gazebo-msgs \
                 ros-melodic-gazebo-plugins \
                 ros-melodic-gazebo-ros \
                 ros-melodic-gazebo-ros-control \
                 ros-melodic-gazebo-ros-pkgs \
                 ros-melodic-joint-state-publisher \
                 ros-melodic-joint-state-publisher-gui \
                 ros-melodic-joint-trajectory-controller
```

### Step 3: Install Python Dependencies

```bash
sudo apt install python-rospkg
```

### Step 4: Set Environment Variables

```bash
sudo nano /etc/environment
```

Add these lines at the end:

```bash
LIDAR_TYPE="A1"
DEPTH_CAMERA_TYPE="AstraProPlus"
MACHINE_TYPE="JetAutoPro"
HOST="/"
MASTER="/"
```

**IMPORTANT: Restart your VM for these changes to take effect.**

### Step 5: Copy Package to ROS Workspace

**Option A: Using GitHub**

1. On your host machine, push the repo to GitHub
2. On your Linux VM:

```bash
cd ~/ros_ws/src
git clone <your-github-repo-url> .
```

**Option B: Manual Copy**

1. Use WinSCP to copy `lab4_jetauto_control` folder
2. Place it in `~/ros_ws/src/`

### Step 6: Build the Package

```bash
cd ~/ros_ws
source devel/setup.bash
catkin_make
source devel/setup.bash
```

### Step 7: Make Script Executable (if needed)

```bash
chmod +x ~/ros_ws/src/lab4_jetauto_control/scripts/jetauto_square_control.py
```

---

## Running the Project

You will need **3 separate terminals**.

### Terminal 1: Start ROS Master

```bash
roscore
```

Leave this running and open a new terminal.

### Terminal 2: Launch JetAuto in Gazebo

```bash
# Source the JetAuto workspace
source ~/jetauto_ws/devel/setup.bash

# Launch Gazebo with the JetAuto robot
roslaunch jetauto_gazebo worlds.launch
```

Wait for Gazebo to fully load. You should see the JetAuto robot in the simulation.

### Terminal 3: Run the Square Pattern Controller

```bash
# Source your ROS workspace
source ~/ros_ws/devel/setup.bash

# Run the controller
rosrun lab4_jetauto_control jetauto_square_control.py
```

**You will see:**

```
==================================================
Press ENTER to start the square pattern...
Press Ctrl+C to exit
==================================================
```

Press **ENTER** to start the movement.

---

## Troubleshooting

### "No module named 'rospkg'" error

```bash
sudo apt install python-rospkg
```

If problem persists:

```bash
sudo apt remove python-rospkg
sudo apt install ros-melodic-desktop-full
# Re-run all Gazebo packages from Step 2
```

### Robot doesn't move when script runs

Check the topic:

```bash
rostopic list | grep cmd_vel
```

Expected output: `/jetauto_controller/cmd_vel`

Verify connection:

```bash
rostopic info /jetauto_controller/cmd_vel
```

### "Permission denied" when running script

```bash
chmod +x ~/ros_ws/src/lab4_jetauto_control/scripts/jetauto_square_control.py
```

### Gazebo won't launch

Check if ROS master is running first (Terminal 1).

Try:

```bash
source /opt/ros/melodic/setup.bash
roslaunch jetauto_gazebo worlds.launch
```

### Robot drifts or doesn't move in straight lines

This is normal in simulation. The Mecanum wheels and physics simulation can cause slight drift. Adjust speeds in the Python script if needed:

```python
# In jetauto_square_control.py
LINEAR_SPEED = 0.1      # Lower for more precision
STRAFE_SPEED = 0.1
ANGULAR_SPEED = 0.5
```

### Need to record a video

Use screen recording in your VM or use OBS Studio:

```bash
sudo apt install obs-studio
```

---

## Assessment Questions

### Question 1: What command did you use to launch the JetAuto robot in Gazebo?

**Answer:**
```bash
roslaunch jetauto_gazebo worlds.launch
```

### Question 2: Which two other launch files were called when you launched worlds.launch?

To find the answer, inspect the launch file:

```bash
cat ~/jetauto_ws/src/jetauto_simulation/jetauto_gazebo/launch/worlds.launch
```

Look for `<include>` tags to see which other launch files are called.

### Question 3: Describe the process of setting up the ROS workspace and creating the package.

**Answer:**
1. Created ROS workspace: `mkdir -p ~/ros_ws/src` and `catkin_make`
2. Created package: `catkin_create_pkg lab4_jetauto_control rospy geometry_msgs`
3. Created `scripts` directory and added Python control script
4. Updated `CMakeLists.txt` to install the Python script
5. Built with `catkin_make`
6. Sourced workspace: `source devel/setup.bash`

### Question 4: How do you estimate that the robot moved roughly 1 meter in each of the steps?

**Answer:**
Using the formula: `distance = speed × time`

- Forward/strafe speed: 0.1 m/s
- To move 1 meter: `time = 1.0 / 0.1 = 10 seconds`

The script calculates duration dynamically:
```python
duration = distance / LINEAR_SPEED  # 1.0 / 0.1 = 10 seconds
```

### Question 5: What challenges did you face with making the robot move in a straight line while turning, and how did you overcome them?

**Answer:**
- **Challenge:** The final step requires simultaneous forward movement AND rotation
- **Solution:** Used the Mecanum wheel capability by publishing both linear and angular velocities in the same Twist message
- The `move_diagonal_with_rotation()` function combines:
  - `linear.x` for forward movement
  - `linear.y` for sideways movement (diagonal)
  - `angular.z` for rotation
- This is possible because Mecanum wheels allow independent control of translation and rotation

---

## Submission Checklist

- [ ] **Source Code:** Link to GitHub repository (private) with all files
- [ ] **Video:** Screen recording of the robot executing the square pattern in Gazebo
- [ ] **Answers File:** Text file (`answers.txt`) with answers to all 5 assessment questions

### Video Requirements

The video should show:
1. Gazebo running with JetAuto robot
2. Running the controller script
3. Pressing ENTER to start
4. Robot completing the square pattern 2 times
5. Robot returning near starting position

### Upload to Blackboard

Due: **Sunday, June 15, 2025 at 11:59 PM**

---

## Movement Parameters (Reference)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `LINEAR_SPEED` | 0.1 m/s | Forward/backward speed |
| `STRAFE_SPEED` | 0.1 m/s | Sideways speed |
| `ANGULAR_SPEED` | 0.5 rad/s | Turning speed (~28.6°/s) |
| `METERS_TO_MOVE` | 1.0 m | Distance for each step |
| `DEGREES_TO_TURN` | 90.0° | Rotation amount |

## File Structure

```
lab4_jetauto_control/
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # ROS package manifest
├── README.md                         # Package documentation
└── scripts/
    └── jetauto_square_control.py     # Main control script
```

---

**Good luck with your project!**
On your Linux VM, run:

  cd ~/ros_ws/src
  git clone https://github.com/kevinhust/aig240project2.git .

  cd ~/ros_ws
  catkin_make
  source devel/setup.bash

  Then follow the running instructions in PROJECT_INSTRUCTIONS.md or the README.
