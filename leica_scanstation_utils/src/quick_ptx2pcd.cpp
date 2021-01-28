/**
 * @file main.cpp
 * @copyright Copyright (c) 2020, FADA-CATEC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "leica_scanstation_utils/LeicaUtils.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quick_ptx2pcd");
    ros::NodeHandle nh;
    ROS_INFO("quick_bin2pcd");

    std::string pc_path;
    if (!nh.getParam("/pointcloud_folder", pc_path))
    {
        pc_path = LeicaUtils::findPointcloudFolderPath();
    }
    ROS_INFO("Pointclouds path: %s", pc_path.c_str());
    
    if(argc<2)
    {
      ROS_INFO("Please specify bin file name to convert as argument. e.g.: \n\trosrun leica_scanstation_ros quick_bin2pcd scan1");
      return 0;
    }
    
    std::string scan_file = argv[1];
    ROS_INFO("Converting file: %s", scan_file.c_str());
    LeicaUtils::ptx2pcd(scan_file);

    std::string output_file = pc_path + scan_file + ".pcd";
    ROS_INFO("File saved to: %s", output_file.c_str());

  return 0;
}
