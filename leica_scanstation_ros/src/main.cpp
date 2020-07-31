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
    if (EventAnalyser::_is_new_image)
    {
      // Publish on topic /image
      sensor_msgs::Image image_msg;
      LeicaNode::getImageMsg(&image_msg);
      LeicaNode::publishVideoImage(g_img_pub, image_msg);
      EventAnalyser::_is_new_image = false;
    }

    // check for scan finished
    if (EventAnalyser::_is_scan_finished)
    {
      // convert file scanned
      std::string scan_file = LeicaNode::_last_file_name;
      ROS_INFO("Converting file: %s", scan_file.c_str());
      LeicaNode::bin2ptx(scan_file);
      LeicaUtils::ptx2pcd(scan_file);
      ROS_INFO("Saved scan pointcloud on: %s", LeicaUtils::getFilePath(scan_file, ".pcd").c_str());
      LeicaNode::publishScanPointcloudFile(g_client, scan_file);
      EventAnalyser::_is_scan_finished = false;
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
  if (nh.getParam("/pointcloud_folder", pc_path))
  {
    ROS_INFO("Pointclouds path: %s", pc_path.c_str());
  }
  else
  {
    pc_path = LeicaUtils::getDefaultPointcloudPath();
    ROS_INFO("Default pointclouds path: %s", pc_path.c_str());
  }
  LeicaUtils::setPointCloudPath(pc_path);

  LeicaNode leica_node = LeicaNode();

  // Get Leica Node publisher in a global copy
  g_pub = leica_node._pub;
  g_img_pub = leica_node._img_pub;

  // Get Leica Node client in a global copy
  g_client = leica_node._client;

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
