
Below are instructions to correctly apply changes to the turtlebot3 model to accurately represent the real world robot.

---------- PLEASE NOTE ---------- 
These steps must be repeated for each device as the modifications are made to files located within git 
submodules. 

This folder should also include:

    - turtlebot3_manipulation_robot.urdf.xacro
    - turtlebot3_waffle_pi_for_open_manipulator.urdf.xacro
    - waffle_manipulator.stl
---------------------------------

---------- CHANGES TO TURTLEBOT3 SUBMODULE ----------
Copy 'waffle_manipulator.stl' into 'turtlebot3/turtlebot3_description/meshes/bases'.

Replace the contents of 'turtlebot3/turtlebot3_description/urdf/turtlebot3_waffle_pi_for_open_manipulator.urdf.xacro' 
with the file of the same name in model_changes.
-------------------------------------------

---------- CHANGES TO TURTLEBOT3_MANIPULATION SUBMODULE ----------
Replace the contents of 'turtlebot3_manipulation/turtlebot3_manipulation_description/urdf/turtlebot3_mnpulation_robot.urdf.xacro' 
with the file of the same name in model_changes.

If, while running the simulation, you encounter a warning about missing collision geometry, then you need to make an
additional change to 'turtlebot_manipulation/turtlebot3_manipulation_description/urdf/open_manipulator_x.urdf.xacro'.
Underneath line 266, paste the following code:

    <collision> 
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.01 0.01 0.01" />
      </geometry>
    </collision>
--------------------------------------------------------

