#include "leica_scanstation_utils/LeicaUtils.h"

std::string LeicaUtils::getDefaultPointcloudPath()
{
  return _default_pointcloud_path;
}

std::string LeicaUtils::findPointcloudFolderPath()
{
  std::string pkg_path = ros::package::getPath("leica_scanstation_utils");

  std::size_t i = pkg_path.find("leica_scanstation_utils");
  pkg_path = pkg_path.substr(0, i - 1);

  _pointcloud_path = pkg_path + "/leica_scanstation_utils/pointclouds/";
  return _pointcloud_path;
}

std::string LeicaUtils::getPointCloudPath()
{
  return _pointcloud_path;
}

void LeicaUtils::setPointCloudPath(std::string pc_path)
{
  _pointcloud_path = pc_path;
}

std::string LeicaUtils::getFilePath(std::string file_name, std::string extension)
{
  std::string file_path = _pointcloud_path;

  file_path += file_name + extension;

  return file_path;
}

std::string LeicaUtils::getFilePath(std::string file_name, std::string extension, int counter)
{
  std::string file_path = _pointcloud_path;
  std::string scan_number = std::to_string(counter);

  file_path += file_name + scan_number + extension;

  return file_path;
}

void LeicaUtils::ptx2pcd(std::string file_name)
{
  ptx_2_pcd converter = ptx_2_pcd();
  converter(getFilePath(file_name, ".ptx"));
}

// Initialization
std::string LeicaUtils::_pointcloud_path = "";
std::string LeicaUtils::_default_pointcloud_path = DEFAULT_PATH;