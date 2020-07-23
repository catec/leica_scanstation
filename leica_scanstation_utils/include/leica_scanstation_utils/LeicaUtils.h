// LeicaUtils.h
#pragma once
#ifndef _LEICAUTILS_H
#define _LEICAUTILS_H

// #include <afx.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include "ptx_2_pcd.hpp"

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

        explicit LeicaUtils() {};
        virtual ~LeicaUtils() {}

        static std::string _pc_path;
        static std::string _default_pc_path;

        static std::string getDefaultPointcloudFolder();
        static std::string findPointcloudFolder();
        static std::string getPointCloudFolder();
        static void setPointCloudFolder(std::string pc_path);
        static std::string getFilePath(std::string file_name, std::string extension);
        static std::string getFilePath(std::string file_name, std::string extension, int counter);
        static void ptx2pcd(std::string file_name);
};