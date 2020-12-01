/**
 * @file LeicaUtils.cpp
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


std::string LeicaUtils::findPointcloudFolderPath()
{
  std::string pkg_path = ros::package::getPath("leica_scanstation_utils");

  std::size_t i = pkg_path.find("leica_scanstation"); // because of mixed / and \ separators
  pkg_path = pkg_path.substr(0, i - 1);

  pointcloud_path_ = pkg_path + "/leica_scanstation/leica_scanstation_utils/pointclouds/";
  return pointcloud_path_;
}

std::string LeicaUtils::getPointCloudPath()
{
  return pointcloud_path_;
}

void LeicaUtils::setPointCloudPath(std::string pc_path)
{
  pointcloud_path_ = pc_path;
}

std::string LeicaUtils::getFilePath(std::string file_name)
{
  std::string file_path = pointcloud_path_;

  file_path += file_name;

  return file_path;
}

std::string LeicaUtils::getFilePath(std::string file_name, std::string extension)
{
  std::string file_path = pointcloud_path_;

  file_path += file_name + extension;

  return file_path;
}

std::string LeicaUtils::getFilePath(std::string file_name, std::string extension, int counter)
{
  std::string file_path = pointcloud_path_;
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
std::string LeicaUtils::pointcloud_path_ = "";
