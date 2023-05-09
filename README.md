# Dissertation

## Installation

This repository contains Git Submodules which, upon cloning, creates empty package folders. These are submodules linking to [ROBOTIS-GIT](https://github.com/ROBOTIS-GIT).
To populate them, the following commands must be executed:
```
git clone <This Repository>
git submodule init
git submodule update
```
Additionally, some system packages are required for these submodules to function as expected:
```
sudo apt install ros-noetic-gazebo* ros-noetic-ros-control* ros-noetic-control* \
ros-noetic-moveit* ros-noetic-industrial-core ros-noetic-dynamixel-sdk \
ros-noetic-dynamixel-workbench* ros-noetic-robotis-manipulator
```

## Updating the Model fo the Turtlebot3
Below are instructions to correctly apply changes to the turtlebot3 model to accurately represent the real world robot.

**Please Note** 
These steps must be repeated for each device as the modifications are made to files located within git 
submodules. 

This folder should also include:

    - turtlebot3_manipulation_robot.urdf.xacro
    - turtlebot3_waffle_pi_for_open_manipulator.urdf.xacro
    - waffle_manipulator.stl

### Changes to the Turtlebot3 Submodule
Copy ***waffle_manipulator.stl*** into ***turtlebot3/turtlebot3_description/meshes/bases***.
Replace the contents of ***turtlebot3/turtlebot3_description/urdf/turtlebot3_waffle_pi_for_open_manipulator.urdf.xacro*** 
with the file of the same name in the **model_changes** folder.


### Changes to the turtlebot3_manipulation submodule
Replace the contents of ***turtlebot3_manipulation/turtlebot3_manipulation_description/urdf/turtlebot3_manpulation_robot.urdf.xacro***
with the file of the same name in the **model_changes** folder.

