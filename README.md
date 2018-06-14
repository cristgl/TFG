# TFG
 This project shows the development of various classification tasks in static and dynamic environments with the Baxter robot. This robot will separate two groups of objects with different colours, positions and sizes, using the Robot Operating System (ROS Kinetic) on Ubuntu 16.04 LTS, MoveIt! and the OpenCV library for the computer vision, based on Maik Heufekes' project (https://github.com/cvr-lab/baxter_pick_and_place).
 
Prerequisites

   Workstation Setup

To set the workstation, the tutorial from Rethink Robotics has been consulted (http://sdk.rethinkrobotics.com/wiki/Workstation_Setup), in this site you can find the installation of Ubuntu, the ROS specific version for that Ubuntu version, the Baxter SDK and the first comunication with the robot, which requires an Ethernet connection.
    Example: Network Connection->Add->Ethernet
    Connection name: Baxter
    IPv4 Settings: Method: Manual
    Address: 169.254.0.10, Netmask: 255.255.0.0, Gateway: 0.0.0.0
   
   MoveIt

   MoveIt! is used for the motion planning, manipulation and collision checking.
    http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial
    Moreover the package moveit_python by Michael Ferguson need to be installed.
   
   Robot arm calibration

   It is recommended to calibrate Baxter's arms for getting more precise results. On the website there are two different calibration routines which are very well described.
    http://sdk.rethinkrobotics.com/wiki/Arm_Calibration
   
   The colour of the objects

   The object recognition can only work if the colours of the objects are known. The colours are defined in the baxter_img.cpp file as low and high HSV values. The best way to get the color values from an object which should be detected is to take pictures with the Baxter camera from it on the workspace in different positions and with different illuminations. 

    $ rosrun image_view image_view image:=/cameras/right_hand_camera/image

   This command opens a window with a stream of images from Baxter's right hand camera and allows to save images by right-clicking on the display window.
    Then the images can be analyzed with the hsvThresholder.py file. Open a new terminal and execute the python file from the location where it is saved.

    $ python hsvThresholder.py frame0000.jpg

Tasks Overview

There are two main classification tasks using a paddle and a wedge as tools, the first one is used in static classification and the other one in dynamic. Both of them are made out of cardboard and it is common that sometimes the objects weight more than the tools, resulting on difficulties at the time of classifying. The clustering algorithm used is K-means with k = 2, it separates the objects by colour and position. This algorithm gives us the centroids of both groups so that we can get the middle point.
