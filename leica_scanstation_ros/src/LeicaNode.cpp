#include "LeicaNode.h"

// LeicaNode::LeicaNode(ros::NodeHandle* nodehandle):_nh(*nodehandle)
LeicaNode::LeicaNode() : _nh(ros::this_node::getName())
{
  ROS_DEBUG("[%s] LeicaNode::LeicaNode()", ros::this_node::getName().data());
  _pub = _nh.advertise<leica_scanstation_msgs::EventerInfo>("/eventer_info", 10);
  _img_pub = _nh.advertise<sensor_msgs::Image>("/image", 10);
  _client = _nh.serviceClient<leica_scanstation_msgs::PointCloudFile>("/publish_clouds");

  _counter = 0;
}

LeicaNode::~LeicaNode()
{
  ROS_DEBUG("[%s] LeicaNode::~LeicaNode()", ros::this_node::getName().data());
}

void LeicaNode::publishEventerInfo(ros::Publisher pub, leica_scanstation_msgs::EventerInfo event_msg)
{
  pub.publish(event_msg);
}

void LeicaNode::publishVideoImage(ros::Publisher pub, sensor_msgs::Image image_msg)
{
  pub.publish(image_msg);
}

void LeicaNode::publishScanPointcloudFile(ros::ServiceClient client, std::string file_name)
{
  leica_scanstation_msgs::PointCloudFile srv;
  srv.request.file_name = file_name;

  client.call(srv);
}

void LeicaNode::searchForScanner(int* numberFound)
{
  int list;
  HXI_FindScanner(5, &list, numberFound);
}

void LeicaNode::connectToScanner()
{
  HXI_OpenScanner();
}

void LeicaNode::startVideo()
{
  HXI_BeginVideo();
}

void LeicaNode::openServices()
{
  _srv0 = _nh.advertiseService("connect", &LeicaNode::connectCb, this);
  _srv1 = _nh.advertiseService("disconnect", &LeicaNode::disconnectCb, this);
  _srv2 = _nh.advertiseService("convert", &LeicaNode::conversionCb, this);
  _srv3 = _nh.advertiseService("move", &LeicaNode::moveCb, this);
  _srv4 = _nh.advertiseService("tilt", &LeicaNode::tiltCb, this);
  _srv5 = _nh.advertiseService("scan_info", &LeicaNode::scaninfoCb, this);
  _srv6 = _nh.advertiseService("scan", &LeicaNode::scanCb, this);
  _srv7 = _nh.advertiseService("cancel", &LeicaNode::cancelCb, this);
  _srv8 = _nh.advertiseService("pause", &LeicaNode::pauseCb, this);
  _srv9 = _nh.advertiseService("resume", &LeicaNode::resumeCb, this);
  _srv10 = _nh.advertiseService("video", &LeicaNode::videoCb, this);
}

int LeicaNode::bin2ptx(std::string file_name)
{
  CString in_file = LeicaUtils::getFilePath(file_name, ".bin").c_str();
  CString out_file = LeicaUtils::getFilePath(file_name, ".ptx").c_str();

  int r = HXI_ConvertToPtx(&in_file, &out_file);

  return r;
}

int LeicaNode::moveMotors(float pan, float tilt)
{
  float angles[2] = { pan, tilt };  // pair of values
  float* ang;

  ang = angles;

  ROS_INFO("Moving to: (%f,%f)", pan, tilt);
  int r = HXI_GotoAngles(ang);
  return r;
}

bool LeicaNode::connectCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Connecting");
  HXI_OpenScanner();

  res.message = "Connecting";
  res.success = true;
  return true;
}

bool LeicaNode::disconnectCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Disconnecting");
  HXI_CloseScanner();

  res.message = "Disconnecting";
  res.success = true;
  return true;
}

bool LeicaNode::conversionCb(leica_scanstation_msgs::PointCloudFile::Request& req,
                             leica_scanstation_msgs::PointCloudFile::Response& res)
{
  CString in_file = LeicaUtils::getFilePath(req.file_name, ".bin").c_str();
  CString out_file = LeicaUtils::getFilePath(req.file_name, ".ptx").c_str();

  int r = HXI_ConvertToPtx(&in_file, &out_file);

  res.success = r == 0 ? true : false;
  res.message = r == 0 ? "look at /pointcloud folder" : "failed, please check file name";
  // todo: mensaje con info mas util
  return true;
}

bool LeicaNode::moveCb(leica_scanstation_msgs::MotorPose::Request& req,
                       leica_scanstation_msgs::MotorPose::Response& res)
{
  int r = HXI_FetchAngles();

  r = moveMotors(req.pan, req.tilt);

  r = HXI_FetchAngles();

  res.message = "Moving";
  res.success = r == 0 ? true : false;
  return true;
}

bool LeicaNode::tiltCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  int enable = req.data == true ? 1 : 0;

  ROS_INFO("Setting tilt to %d", enable);
  HXI_EnableTilt(enable);

  res.message = "Got it";
  res.success = false;
  return true;
}

bool LeicaNode::scaninfoCb(leica_scanstation_msgs::PointCloudFile::Request& req,
                           leica_scanstation_msgs::PointCloudFile::Response& res)
{
  CString file_path = LeicaUtils::getFilePath(req.file_name, ".bin").c_str();

  int* size;
  size = new int[2];
  float* window;
  window = new float[4];

  HXI_ReadScanInfo(&file_path, window, size);

  res.message = "Got it";
  res.success = true;

  return true;
}

bool LeicaNode::scanCb(leica_scanstation_msgs::Scan::Request& req, leica_scanstation_msgs::Scan::Response& res)
{
  int* size;
  size = new int[2];
  float* window;
  window = new float[4];

  CString file_path = LeicaUtils::getFilePath(req.file_name, ".bin", _counter).c_str();

  ROS_INFO("Saving scan on: %S", file_path);

  size[0] = req.vertical_res;
  size[1] = req.horizontal_res;
  window[0] = req.pan_center;
  window[1] = req.tilt_center;
  window[2] = req.width;
  window[3] = req.height;

  // Stop video streaming before scan
  HXI_EndVideo();

  // Tell user scan range
  float angles[2];
  angles[0] = req.width;
  angles[1] = req.height;
  HXI_FetchRange(angles);

  // Start scan
  HXI_FetchScan(window, size, &file_path);

  // Save file name to be converted
  _last_file_name = req.file_name + std::to_string(_counter);

  _counter++;

  res.message = "Scanning";
  res.success = true;
  return true;
}

bool LeicaNode::cancelCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Cancel requested ");
  HXI_Cancel();

  _counter--;  // Last scan was cancelled

  res.message = "Canceled";
  res.success = true;
  return true;
}

bool LeicaNode::pauseCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Pause requested ");
  HXI_Pause();

  res.message = "Paused";
  res.success = true;
  return true;
}

bool LeicaNode::resumeCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("Resume requested ");
  HXI_Resume();

  res.message = "Resumed";
  res.success = true;
  return true;
}

bool LeicaNode::videoCb(leica_scanstation_msgs::MotorPose::Request& req,
                        leica_scanstation_msgs::MotorPose::Response& res)
{
  ROS_INFO("Video starting");
  HXI_BeginVideo();

  // ROS_INFO("Video angles");
  float angles[2] = { req.pan, req.tilt };  // pair of values
  float* ang;
  ang = angles;
  // HXI_FetchVideoAngles(ang);

  ROS_INFO("Video moving");
  HXI_MoveVideo(ang);

  res.message = "Resumed";
  res.success = true;
  return true;
}

void LeicaNode::getImageMsg(sensor_msgs::Image* img_msg)
{
  HxiImageT img;                          // image for holding scanner data
  img.size[0] = WIDTH + (3 * WIDTH & 3);  // be multiple of 4
  img.size[1] = HEIGHT;
  img.data = (HxiPixelT*)malloc(img.size[0] * img.size[1] * sizeof(HxiPixelT));
  std::memset(img.data, 255, img.size[0] * img.size[1] * sizeof(HxiPixelT));  // initialize on 255

  HXI_FetchVideo(&img);

  if (img.data != NULL)
  {
    // Assemble ros img_msg
    img_msg->height = HEIGHT;
    img_msg->width = WIDTH;
    img_msg->encoding = sensor_msgs::image_encodings::BGR8;
    img_msg->step = NUM_CHANNELS * WIDTH;

    // img_msg->data.resize(WIDTH*HEIGHT*NUM_CHANNELS);

    // reassigning data from scanner to ros img_msg
    for (int i = 0; i < (img.size[0] * img.size[1]); i++)
    {
      img_msg->data.push_back(img.data[i].b);
      img_msg->data.push_back(img.data[i].g);
      img_msg->data.push_back(img.data[i].r);
    }

    HXI_FreeImage(&img);
  }
}

// Initialization
std::string LeicaNode::_last_file_name = "";