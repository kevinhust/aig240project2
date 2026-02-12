# Project 2 - Moving JetAuto in Gazebo

AIG240 Robotics - Seneca Polytechnic

## Overview

This package contains a Python ROS node that controls the JetAuto robot to move in a 1-meter square pattern in Gazebo simulation.

## Prerequisites

- Ubuntu with ROS Melodic installed
- JetAuto workspace (`jetauto_ws`) already set up
- Python 2.7 (comes with ROS Melodic)

## Setup Instructions

### Step 1: Install Gazebo

On your Linux VM, run:

```bash
sudo apt update
sudo apt install gazebo9 libgazebo9-dev
```

Verify Gazebo installation:

```bash
gazebo
```

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

### Step 3: Install Python Dependencies (if needed)

```bash
sudo apt install python-rospkg
```

### Step 4: Set Environment Variables

Edit the environment file:

```bash
sudo nano /etc/environment
```

Add these lines:

```bash
LIDAR_TYPE="A1"
DEPTH_CAMERA_TYPE="AstraProPlus"
MACHINE_TYPE="JetAutoPro"
HOST="/"
MASTER="/"
```

**Restart your VM for changes to take effect.**

### Step 5: Copy This Package to Your ROS Workspace

If using GitHub:
1. Push this folder to your GitHub repository
2. On your Linux VM, clone to `~/ros_ws/src/`:

```bash
cd ~/ros_ws/src
git clone <your-repo-url>
```

Or manually copy the `lab4_jetauto_control` folder to `~/ros_ws/src/`

### Step 6: Source and Build

```bash
cd ~/ros_ws
source devel/setup.bash
catkin_make
source devel/setup.bash
```

## Running the Project

### Terminal 1: Start ROS Master

```bash
roscore
```

### Terminal 2: Launch JetAuto in Gazebo

```bash
# Source the JetAuto workspace
source ~/jetauto_ws/devel/setup.bash

# Launch Gazebo with JetAuto
roslaunch jetauto_gazebo worlds.launch
```

Wait for Gazebo to fully load with the JetAuto robot.

### Terminal 3: Run the Square Pattern Controller

```bash
# Source your ROS workspace
source ~/ros_ws/devel/setup.bash

# Run the controller
rosrun lab4_jetauto_control jetauto_square_control.py
```

Press **ENTER** when prompted to start the square pattern.

The robot will execute the pattern **2 times**:
1. Forward 1m
2. Left strafe 1m
3. Turn clockwise 90°
4. Right strafe 1m
5. Diagonal movement with rotation back to start

## Troubleshooting

### "No module named 'rospkg'" error

```bash
sudo apt install python-rospkg
```

If problem persists:
```bash
sudo apt remove python-rospkg
sudo apt install ros-melodic-desktop-full
```

### Robot doesn't move

Check if the topic is correct:
```bash
rostopic list | grep cmd_vel
```

You should see `/jetauto_controller/cmd_vel`

### Script not executable

```bash
chmod +x ~/ros_ws/src/lab4_jetauto_control/scripts/jetauto_square_control.py
```

## Square Pattern Details

```
Position Format: (x, y, yaw_degrees)

Step 1: (0, 0, 0°)  → (1, 0, 0°)   [Forward 1m, facing direction of travel]
Step 2: (1, 0, 0°)  → (1, 1, 0°)   [Left strafe 1m, no turning]
Step 3: (1, 1, 0°)  → (1, 1, -90°) [Turn clockwise 90°]
Step 4: (1, 1, -90°) → (0, 1, -90°) [Right strafe 1m, facing inside square]
Step 5: (0, 1, -90°) → (0, 0, 0°)  [Diagonal with rotation]

Repeat 2 times total.
```

## Movement Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| LINEAR_SPEED | 0.1 m/s | Forward/backward speed |
| STRAFE_SPEED | 0.1 m/s | Sideways speed |
| ANGULAR_SPEED | 0.5 rad/s | Turning speed |

These can be adjusted in `jetauto_square_control.py` if needed.

## Assessment Questions

1. **Launch command:** `roslaunch jetauto_gazebo worlds.launch`

2. **Launch files called by worlds.launch:**
   - Inspect with: `cat ~/jetauto_ws/src/jetauto_simulation/jetauto_gazebo/launch/worlds.launch`

3. **Workspace setup process:**
   - Created `~/ros_ws/src/` directory
   - Created package with `catkin_create_pkg`
   - Built with `catkin_make`
   - Sourced with `source devel/setup.bash`

4. **1 meter estimation:**
   - Using formula: `time = distance / speed`
   - At 0.1 m/s, moving for 10 seconds = 1 meter
   - Distance = speed × time

5. **Challenges with straight line while turning:**
   - Mecanum wheels allow simultaneous translation and rotation
   - Used `move_diagonal_with_rotation()` function
   - Combined linear velocity with angular velocity in same Twist message

## Submission Checklist

- [ ] Video of robot executing the square pattern in Gazebo
- [ ] Link to project folder (GitHub private repo)
- [ ] Text file with assessment question answers
