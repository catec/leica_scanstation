# Leica Scanstation ROS

This package is created to control in PC, throught ROS interface, the Leica Scanstation C5.
Tested on ROS Melodic for Windows 10.

Due to Leica's libraries dependencies, the source code development is done under Windows 10. 

Once the program is released, it could be executed in either Windows or Linux (with tools such us Wine).

# Dependencies #

- [ROS](http://wiki.ros.org/Installation/Windows)

Recomended: `ros-melodic-desktop_full`

# Set up #
0. Mandatory: Windows 10
1. Create the following workspace *catkin_ws*
2. Clone repo on your workspace
3. Place Leica libraries where they can be reached by this package:

   - HxiDefinitions.h -> `c:/opt/leica/include`

   - HxiScanner.lib -> `c:/opt/leica/lib`

   - Leica's DLL files -> `catkin_ws/devel/lib/leica_scanstation_ros`  
   (this is the path where leica_scanstation_ros_node.exe is located) 

4. We created scripts to do this: 

        cd leica_scanstation\leica_scanstation_ros\config
        SetLibraryFiles.bat

5. Start development here. When ready, compile it.

        cd catkin_ws
        catkin_make
        devel\setup.bat

6. Before running, move DLL files

        cd leica_scanstation\leica_scanstation_ros\config
        SetDLLFiles.bat

7. Program could also be executed under linux OS, so once finished, release it:
   
        cd leica_scanstation\leica_scanstation_ros\config
        ExportProgram.bat

   - Files will be saved under `leica_scanstation\leica_scanstation_ros_release` folder

# Usage #

        roslaunch leica_scanstation_ros start.launch

Use [ROS services](#utils-for-w10-cmd) to control, move and start scanning on the Scanstation

        rosservice list

# Code API #