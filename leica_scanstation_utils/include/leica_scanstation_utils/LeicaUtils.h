/**
 * @file LeicaUtils.h
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
 
#pragma once
#ifndef _LEICAUTILS_H
#define _LEICAUTILS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include "leica_scanstation_utils/ptx_2_pcd.h"

#endif

class LeicaUtils
{
public:
    /**
     * @brief Search the absolute path for leica_scanstation_utils/pointclouds folder
     *
     * @return std::string
     */
    static std::string findPointcloudFolderPath();

    /**
     * @brief Get the Point Cloud Path object
     *
     * @return std::string
     */
    static std::string getPointCloudPath();

    /**
     * @brief Set the Point Cloud Path object
     *
     * @param pc_path
     */
    static void setPointCloudPath(std::string pc_path);

    /**
     * @brief Get the path to file relative to pointcloud path defined
     *
     * @param[in] file_name, that includes extension
     * @return std::string
     */
    static std::string getFilePath(std::string file_name);

    /**
     * @brief Get the path to file relative to pointcloud path defined
     *
     * @param[in] file_name
     * @param[in] extension
     * @return std::string
     */
    static std::string getFilePath(std::string file_name, std::string extension);

    /**
     * @brief Get the path to file relative to pointcloud path defined. If file name ends with number, you could specify
     * it separately.
     *
     * @param[in] file_name
     * @param[in] extension
     * @param[in] counter
     * @return std::string
     */
    static std::string getFilePath(std::string file_name, std::string extension, int counter);

    /**
     * @brief Convert file given from .ptx format to PCL .pcd format.
     *        \n Automatically saved in same path than input cloud.
     *
     * @param file_name
     */
    static void ptx2pcd(std::string file_name);

private:
    /**
     * @brief This class is not meant to be instantiated.
     *
     */
    LeicaUtils(){};

    /**
     * @brief This class is not meant to be instantiated.
     *
     */
    ~LeicaUtils()
    {
    }
    /** @brief  Absolute Path to leica_scanstation_utils/pointclouds folder   */
    static std::string pointcloud_path_;
};
