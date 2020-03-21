Chris Wheatley
University of Maryland (College Park)
ENPM661 Spring 2020
Dr. Monfaredi
Project #3 - Phase #2

Project Description:
These programs will leverage the A* algorithm to explore an action space and naviagte obstacles to generate an optimal path, for a rigid robot.

The project contains the following files: 
	-Astar_rigid.m
	-obstacleCheckRigid.m 
	-changeAngle.m
	-point_to_line.m
	-astar_wheatley_simulation_video.mp4
	-README.txt
	
User Instructions:
1) Ensure that MATLAB is installed on your machine (2017b or later).

2) Download all project contents and unzip the "proj3p2_24__matlab.zip" contents into a newly created local directory.

3) In MATLAB, change the working directory to the working directory created in step #2.

4) For point robots:
	-Run "Astar_rigid.m"
	-A prompt will appear in the MATLAB command window asking you to enter location and orientation of initial node. Enter it between brackets and with column separation (Ex: "[5,5,60]")
	-Another prompt will appear asking to enter location of goal node. Enter it between brackets and with column separation (Ex: "[150,150]")
	-Another prompt will appear asking you to enter the robot radius. Enter an integer.
	-Another prompt will appear asking you to enter the robot's obstacle clearance (unit length). Enter an integer.
	-Another prompt will appear asking you to enter the robot's step/movement distance (unit length). Enter an integer between 1 and 10.
	-Another prompt will appear asking you to enter the robot's movement angle between consecutive actions. Enter a value in degrees (Ex: "30")
	-If there is conflict with these points, the program will prompt you again, so please enter new values.

5) A MATLAB figure (.fig) should appear and be updating in real time, drawing lines from node to node in a branching motion.  Obstacles are outlined in blue.
6) Once the program finishes running, node exploration should stop, and the optimal path should appear colored in red.
7) The pink circle is the start point and the pink triangle is the goal point.  If you would like to zoom in, select the + magnifying glass the top ribbon of figure and use your mouse to zoom.
