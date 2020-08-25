# LEICA SCANSTATION

This repository contains packages created to control in PC, throught ROS interface, the Leica Scanstation C5. 

![leica](scanstationC5.jpg)

Due to Leica's libraries dependencies, the source code relative to SDK have been developed under **Windows 10**. 

For that reason, *leica_scanstation_ros* will only compile on Windows OS.

However, once the program is released, it could be executed in either **Windows or Linux** (with tools such us Wine).

In the following sections, it is described which packages are included and an explanation of how to use them.

For **development purposes**, please refer to *leica_scanstation_ros* documentation. 

It is involved in the ROSIN project [Large_3D_inspection](http://wiki.ros.org/large_3d_inspection)

## Package List
- **leica_scanstation_ros**. Based on Leica SDK, this package contains functions to communicate with the device using a computer. Note that this package is meant to be compiled and executed in Windows. It includes a ROS node with the following options:
    - operating the device through ROS services. 
    - to read and understand the information sent by the scanstation which is published in a ROS topic. 
    - to view the video captured by the device which is published as images in another ROS Topic.

- **leica_scanstation_ros_release**. This is a container for the application already compiled and the library files needed to run it. The aim is to be able to execute the program in both Windows and Linux (this one requires additional tools such as wine), avoiding compilation errors.

- **leica_scanstation_msgs**. This package defines types of messages and services that permit interpretation of the scanner information.
It is OS independent.

- **leica_scanstation_utils**. This package contains common functions that assist in the program's execution independently of the OS used. Its main purpose is to define the paths for generating and collecting the point clouds. 


## Dependencies 
- [ROS for Windows](http://wiki.ros.org/Installation/Windows) // [ROS for Ubuntu](http://wiki.ros.org/Installation/Ubuntu)
- [Wine for ubuntu](https://tecadmin.net/install-wine-on-ubuntu/)
- Eigen3

        sudo apt-get install libeigen3-dev

## Usage
### Windows
1. Create a workspace, clone and compile this repo

        mkdir catkin_ws\src && cd catkin_ws\src
        git clone https://github.com/fada-catec/leica_scanstation.git
        cd ..
        catkin_make
        devel\setup.bat

2. Execute program

        roslaunch leica_scanstation_ros start.launch

3. Use ROS services to control, move and start scanning on the Scanstation

        rosservice list

### Ubuntu (using Wine)
1. Create a workspace and clone this repo

        mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
        git clone https://github.com/fada-catec/leica_scanstation.git

2. Remove package *leica_scanstation_ros* to avoid compilation errors.

        rm -r leica_scanstation/leica_scanstation_ros

4. Compile

        cd ~/catkin_ws
        catkin_make

5. Start roscore in one terminal and execute app in another

        roscore
        cd ~/catkin_ws/src/leica_scanstation/leica_scanstation_ros_release
        wineconsole leica_scanstation_ros_node.exe

6. Use ROS services to control, move and start scanning on the Scanstation

        rosservice list

## Code API

[Topics, Services, Params](http://wiki.ros.org/leica_scanstation#Code_API)

Read documentation on

`leica_scanstation_ros/doc/html/index.html`

`leica_scanstation_utils/doc/html/index.html`

##  Acknowledgement ##
***
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 

## Help
Ines M. Lara - imlara@catec.aero