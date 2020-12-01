# Leica Scanstation ROS

This package is created to control in PC, throught ROS interface, the Leica Scanstation C5.
Tested on ROS Melodic for Windows 10.

Due to Leica's libraries dependencies, the source code development is done under Windows 10. 

Once the program is released, it could be executed in either Windows or Linux (with tools such us Wine).

# Dependencies #

- [ROS](http://wiki.ros.org/Installation/Windows)

Recomended: `ros-melodic-desktop_full`

- Leica HXI SDK.

Note: To get your HXI SDK contact your local Leica supplier.


# Set up #
0. Mandatory: Windows 10
1. Create a workspace e.g: *catkin_ws*

        mkdir catkin_ws\src && cd catkin_ws\src

2. Clone repo on your workspace

        git clone https://github.com/fada-catec/leica_scanstation.git

3. Place HXI SDK files on a reachable path. Recommended: create a directory on `c:/opt/hxi_scanner` and copy the content of your HXI SDK into the directory
        
4. Set path to SDK files

        setx HXI_SCANNER_INCLUDE_PATH c:/opt/hxi_scanner/Include
        setx HXI_SCANNER_BINARY_PATH c:/opt/hxi_scanner/Release

5. Compile your workspace. 

        cd catkin_ws
        catkin_make
        devel\setup.bat

# Usage #

        roslaunch leica_scanstation_ros start.launch

Use ROS services to control, move and start scanning on the Scanstation. E.g: Try to move your pan and tilt

        rosservice call /leica/move 3.141 0.785 

# Export program #
The ROS node created for this package can be executed in a Linux distribution, using tools such as [Wine](https://www.winehq.org/). If you plan to do this, it will be necessary to have the related libraries to run the program. Compile the program setting export option. Move the executable file (.exe) and the libraries to your linux system.
   
        setx LEICA_EXPORT_PROGRAM ON
        catkin_make

Files will be exported on your package folder: `catkin_ws\...\leica_scanstation_ros\export`

# Code API #

Read documentation `leica_scanstation_ros/doc/html/index.html`