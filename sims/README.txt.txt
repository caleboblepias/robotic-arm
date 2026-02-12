README.txt

	The UserInterfacePanel.mlapp application can be found under the App Design Folder and simulates a 4-DOF robotic ride mechanism using MATLAB.
All kinematics, Jacobians, trajectory generation, timing laws, control, and error calculations are manually implemented in the provided code.

The only external tool used is Peter Corke’s Robotics Toolbox, which is responsible solely for animating the robot based on computed joint angles.
No kinematic or control functions from the toolbox are used — only its visualization utilities.

The simulation is controlled through a MATLAB App Designer GUI, which allows the user to:

Select a predefined trajectory path
Start the simulation
Stop the simulation using the Emergency Stop Button
The GUI does not compute kinematics or control itself—it simply triggers the control loop and displays the results.

When the user selects a path and presses Start, the GUI launches the control loop.
The following computations are performed manually in the code:

Geometric curve generation for the spatial path
Quintic polynomial time-scaling
Forward kinematics via DH parameters
Geometric Jacobian
Angle–axis orientation error
Task-space PD control
Damped least-squares inverse kinematics
Joint angle integration using Euler’s method
The control loop outputs a full time history of joint angles, end-effector pose, and error metrics.

The GUI displays the following simulation results:

X, Y, Z position vs. time
Desired vs. Actual position magnitude
Position error vs. time
Orientation (angle–axis) error vs. time
Joint angles vs. time for all 4 DOF