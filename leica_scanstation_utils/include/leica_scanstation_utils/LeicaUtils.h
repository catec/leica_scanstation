// LeicaUtils.h
#pragma once
#ifndef _LEICAUTILS_H
#define _LEICAUTILS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include "leica_scanstation_utils/ptx_2_pcd.h"

#endif 

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define DEFAULT_PATH "C:/Users/inesPC/catkin_ws/src/leica_scanstation_utils/pointclouds/"

#elif __linux__
#define DEFAULT_PATH "/home/catec/catkin_ws/src/leica_scanstation_utils/pointclouds/"

#elif __unix__ // all unices not caught above
#define DEFAULT_PATH "/home/catec/catkin_ws/src/leica_scanstation_utils/pointclouds/"

#endif


class LeicaUtils {

public:

    /**
     * @brief Search the absolute path for leica_scanstation_utils/pointclouds folder 
     * 
     * @return std::string 
     */
    static std::string findPointcloudFolderPath();

    /**
     * @brief Get the Default Pointcloud Path object
     * 
     * @return std::string 
     */
    static std::string getDefaultPointcloudPath();

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
     * @param[in] file_name 
     * @param[in] extension 
     * @return std::string 
     */
    static std::string getFilePath(std::string file_name, std::string extension);

    /**
     * @brief Get the path to file relative to pointcloud path defined. If file name ends with number, you could specify it separately.
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
    LeicaUtils() {};

    /**
     * @brief This class is not meant to be instantiated.
     * 
     */
    ~LeicaUtils() {}


    /** @brief  Absolute Path to leica_scanstation_utils/pointclouds folder   */
    static std::string _pointcloud_path;

    /** @brief  Default Absolute Path to leica_scanstation_utils/pointclouds folder. It is OS dependent  */
    static std::string _default_pointcloud_path;
};