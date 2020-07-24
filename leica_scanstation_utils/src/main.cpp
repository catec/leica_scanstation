#include "leica_scanstation_utils/LeicaUtils.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leica_utils");
	ros::NodeHandle nh;

    std::string pc_path = LeicaUtils::findPointcloudFolderPath();
	ROS_INFO("Pointclouds path: %s", pc_path.c_str());

    nh.setParam("/pointcloud_folder", pc_path);

    return 0;
}