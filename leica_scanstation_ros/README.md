# Leica Scanstation SDK control

This package is created to control in PC, throught ROS interface, the Leica Scanstation C5.
Tested on ROS Melodic for Windows 10.

Due to Leica's libraries dependencies, the source code development is done under Windows 10. 

Once the program is released, it could be executed in either Windows or Linux (with tools such us Wine).
Both cases requires the listed Dependencies to be installed. 

- For development purposes, please refer to Set Up and Documentation.

- For using and testing the system, please refer to Usage in the correspondent OS and take a look at Utils for cmd.

# Dependencies #

- ROS

- Eigen3

Clone and compile these repos:

- [leica_scanstation_msgs](https://bitbucket.org/ayr_catec/leica_scanstation_msgs/src/master/)

- [leica_scanstation_utils](https://bitbucket.org/ayr_catec/leica_scanstation_utils/src/master/)

# Set up #
0. Mandatory: Windows 10
1. Create the following workspace *catkin_ws*
2. Clone repo on your workspace
3. Place Leica libraries where it can be reached by this package:

   - HxiDefinitions.h -> `c:/opt/leica/include`

   - HxiScanner.lib -> `c:/opt/leica/lib`

   - Leica's DLL files -> `catkin_ws/devel/lib/leica_scanstation_ros`  
   (this is the path where leica_scanstation_ros_node.exe is located) 

4. We created scripts to do this: 

        cd catkin_ws\src\leica_scanstation_ros\config
        SetLibraryFiles.bat

5. Compile

        cd catkin_ws
        catkin_make

6. Before running, move DLL files

        cd catkin_ws\src\leica_scanstation_ros\config
        SetDLLFiles.bat

7. Ready for development. Once finished, release it:
   
   - Compile

                cd catkin_ws && catkin_make

   - Move to repo directory and delete /release folder
        
                cd src\leica_scanstation_ros && rmdir /q/s release

   - Move to config folder and export program

                cd config && ExportProgram.bat

   - Files will be saved under `/release` folder


# Usage #
## Windows

        roslaunch leica_scanstation_ros start.launch

Use [ROS services](#utils-for-w10-cmd) to control, move and start scanning on the Scanstation

        rosservice list

## Linux
Clone this package OUT OF WORKSPACE as it won't compile on Linux.

Start LeicaUtils node to save on Param Server pointcloud folder path `/pointcloud_folder`

        roscore
        rosrun leica_scanstation_utils main 

Move to `/release` folder and run

        wineconsole leica_scanstation_ros_node.exe

Sometimes Wine fails to find DLLs. They are placed on `/extra_dll`. Move them to /release folder.

Use [ROS services](#utils-for-w10-cmd) to control, move and start scanning on the Scanstation

        rosservice list

## Utils for W10 cmd
Info:

        rostopic echo /eventer_info
        rosrun image_view image_view image:=/image

Service calls:

        rosservice call /leica/connect
        rosservice call /leica/convert "scan00"
        rosservice call /leica/move 0 0
        rosservice call /leica/scan_info "scan00"
        rosservice call /leica/scan "sala" 100 100 0 0 0.2 0.2
