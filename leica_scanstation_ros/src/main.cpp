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

#include "LeicaNode.h"
#include "EventAnalyser.h"
#include "leica_scanstation_utils/LeicaUtils.h"

// Global publisher
ros::Publisher g_pub, g_img_pub;
ros::ServiceClient g_client;

int ScannerEventHandler(LeicaEventPtr Eventer)
{
  // Publish on topic /eventer_info
  leica_scanstation_msgs::EventerInfo event_msg;
  EventAnalyser::assemblePublishMsg(&event_msg, Eventer);
  LeicaNode::publishEventerInfo(g_pub, event_msg);

  // Analyse event result
  if (EventAnalyser::isError(Eventer))
    ROS_ERROR("ERROR!!");
  else
  {
    std::string print_info = EventAnalyser::getStringResult(Eventer);
    if (print_info != "")
      ROS_INFO("%s", print_info.c_str());

    // check for new image
    if (EventAnalyser::is_new_image_)
    {
      // Publish on topic /image
      sensor_msgs::Image image_msg;
      LeicaNode::getImageMsg(&image_msg);
      LeicaNode::publishVideoImage(g_img_pub, image_msg);
      EventAnalyser::is_new_image_ = false;
    }

    // check for scan finished
    if (EventAnalyser::is_scan_finished_)
    {
      // convert file scanned
      std::string scan_file = LeicaNode::last_file_name_;
      ROS_INFO("Converting file: %s", scan_file.c_str());
      LeicaNode::bin2ptx(scan_file);
      LeicaUtils::ptx2pcd(scan_file);
      ROS_INFO("Saved scan pointcloud on: %s", LeicaUtils::getFilePath(scan_file, ".pcd").c_str());
      LeicaNode::publishScanPointcloudFile(g_client, scan_file);
      EventAnalyser::is_scan_finished_ = false;
    }
  }

  return 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leica");
  ros::NodeHandle nh;
  ROS_INFO("MAIN");

  // Tell Leica Node where should store scans and pointclouds
  // The param /pointcloud_folder is set by a node on LeicaUtils.
  // Run it using: 	rosrun leica_scanstation_utils main
  std::string pc_path;
  if (!nh.getParam("/pointcloud_folder", pc_path))
  {
    pc_path = LeicaUtils::findPointcloudFolderPath();
  }
  ROS_INFO("Pointclouds path: %s", pc_path.c_str());
  
  LeicaNode leica_node = LeicaNode();

  // Get Leica Node publisher in a global copy
  g_pub = leica_node.pub_;
  g_img_pub = leica_node.img_pub_;

  // Get Leica Node client in a global copy
  g_client = leica_node.client_;

  int serial_number = 1260916;  // not needed
  int nfound;

  // Scan the network to get scanners
  leica_node.searchForScanner(&nfound);
  ROS_INFO("found %d scanners", nfound);

  if (nfound > 0)
  {
    // Scanner responds to commands by sending user events
    // We use the event handler to analyse this response
    ROS_INFO("Setting event handler");
    EventAnalyser::setEventHandler(ScannerEventHandler);

    // Connect to scanner
    ROS_INFO("Connecting");
    leica_node.connectToScanner();

    // When connected, scanner start publishing video images
    ROS_INFO("Starting video");
    leica_node.startVideo();
  }

  // Start ros services to control scanner
  leica_node.openServices();

  ros::spin();

  return 0;
}
