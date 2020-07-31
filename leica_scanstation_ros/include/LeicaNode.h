// LeicaNode.h
#pragma once
#ifndef _LEICANODE_H
#define _LEICANODE_H

#include <afx.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include <leica_scanstation_msgs/EventerInfo.h>
#include <leica_scanstation_msgs/PointCloudFile.h>
#include <leica_scanstation_msgs/MotorPose.h>
#include <leica_scanstation_msgs/Video.h>
#include <leica_scanstation_msgs/Scan.h>

#include "leica_scanstation_utils/LeicaUtils.h"

#include "HxiDefinitions.h"

#endif

#define RAW_WIDTH 320
#define WIDTH RAW_WIDTH + (3 * RAW_WIDTH & 3)  // be multiple of 4
#define HEIGHT 240
#define NUM_CHANNELS 3

class LeicaNode
{
public:
  /**
   * @brief Construct a new Leica Node object
   *
   */
  LeicaNode();

  /**
   * @brief Destroy the Leica Node object
   *
   */
  ~LeicaNode();

  ros::NodeHandle _nh;
  ros::Publisher _pub, _img_pub;
  ros::ServiceClient _client;
  ros::ServiceServer _srv0, _srv1, _srv2, _srv3, _srv4, _srv5, _srv6, _srv7, _srv8, _srv9, _srv10;

  /** @brief Counter scans */
  int _counter;

  /** @brief Store last scan file name  */
  static std::string _last_file_name;

  /**
   * @brief Start ROS services to control Leica Scanstation C5.
   *
   */
  void openServices();

  /**
   * @brief Move Leica motors to specified pan and tilt values. Return 0 if no error.
   *
   * @param pan
   * @param tilt
   * @return int
   */
  int moveMotors(float pan, float tilt);

  /**
   * @brief Convert scan binary file to pointcloud format PTX. Return 0 if no error.
   *        \n Converted file name and directory is the same as the input.
   *
   * @param file_name
   * @return int
   */
  static int bin2ptx(std::string file_name);

  /**
   * @brief Use the specified service client to send file request.
   *
   * @param[in] client
   * @param[in] file_name
   */
  static void publishScanPointcloudFile(ros::ServiceClient client, std::string file_name);

  /**
   * @brief Use the specified publisher to publish the EventerInfo message.
   *
   * @param[in] pub
   * @param[in] event_msg
   */
  static void publishEventerInfo(ros::Publisher pub, leica_scanstation_msgs::EventerInfo event_msg);

  /**
   * @brief Use the specified publisher to publish the Image message.
   *
   * @param[in] pub
   * @param[in] image_msg
   */
  static void publishVideoImage(ros::Publisher pub, sensor_msgs::Image image_msg);

  /**
   * @brief Get an Image message from the last video image in the device.
   *
   * @param[out] img_msg
   */
  static void getImageMsg(sensor_msgs::Image* img_msg);

  /**
   * @brief Scan the network to search for Leica Scanstations.
   *
   */
  void searchForScanner(int* numberFound);

  /**
   * @brief Connect PC with Leica Scanstation device.
   *
   */
  void connectToScanner();

  /**
   * @brief Start video in Leica Scanstation device. Images are published in ROS topic.
   *
   */
  void startVideo();

  /**
   * @brief Service callback to connect PC with Leica Scanstation device.
   *
   */
  bool connectCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Service callback to disconnect PC from Leica Scanstation device.
   *
   */
  bool disconnectCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Service callback to request conversion from binary scan file to pointcloud format.
   *
   */
  bool conversionCb(leica_scanstation_msgs::PointCloudFile::Request& req,
                    leica_scanstation_msgs::PointCloudFile::Response& res);

  /**
   * @brief Service callback to move Leica Scanstation device to selected position.
   *
   */
  bool moveCb(leica_scanstation_msgs::MotorPose::Request& req, leica_scanstation_msgs::MotorPose::Response& res);

  /**
   * @brief Service callback to request scan info and parameters of indicated file.
   *
   */
  bool scaninfoCb(leica_scanstation_msgs::PointCloudFile::Request& req,
                  leica_scanstation_msgs::PointCloudFile::Response& res);

  /**
   * @brief Service callback to start scan process with Leica Scanstation device.
   *
   */
  bool scanCb(leica_scanstation_msgs::Scan::Request& req, leica_scanstation_msgs::Scan::Response& res);

  /**
   * @brief Service callback to cancel scan process in Leica Scanstation device.
   *
   */
  bool cancelCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Service callback to pause scan process in Leica Scanstation device.
   *
   */
  bool pauseCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Service callback to resume scan process in Leica Scanstation device.
   *
   */
  bool resumeCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Service callback to enable tilt of Leica Scanstation device.
   *
   */
  bool tiltCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  /**
   * @brief Service callback to start video in Leica Scanstation device.
   * Images are published in ROS topic.
   *
   */
  bool videoCb(leica_scanstation_msgs::MotorPose::Request& req, leica_scanstation_msgs::MotorPose::Response& res);
};