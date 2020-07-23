# Leica Scanstation Utils
This package includes some Utils to work with package `leica_scanstation_ros`.

This package have also the folder `/pointclouds` where scans and pointclouds will be stored. 

# Windows #
Compile to get `leica_scanstation_utils.dll`. This file is necessary to work on wine together with package leica_scanstation_ros.

# Linux # 
Compile to get access to pointcloud folder. 

        catkin_make

Run script to store /pointclouds folder path on ROS Param Server

        rosrun leica_scanstation_utils main

In this package there is also a script to convert pointclouds from Leica format to standard `.pcd`

        rosrun leica_scanstation_utils ptx_2_pcd.py pointcloud_name



## Dependencies
- ROS
- Python 
- PCL

        sudo apt-get install ros-distro-pcl-tools
