/**
 * @file EventAnalyser.cpp
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

#include "EventAnalyser.h"

void EventAnalyser::setEventHandler(notify event_handler)
{
    notify event_function;           // pointer-to-function
    event_function = event_handler;  // assign event handler

    HXI_SetObserver(event_function);  // tell Leica who is event handler
}

int EventAnalyser::ScannerEventHandler(HxiEventT* Eventer)
{
    return 0;
}

std::string EventAnalyser::getStringResult(HxiEventT* Eventer)
{
    std::string string_result = "";
    // Analyse Event Finalized
    if (Eventer->mode == HXI_DONE_EVENT)
    {
        string_result = readDoneEventCmdAndValues(Eventer->cmd, Eventer->value);

        // check for scan finished
        if (Eventer->cmd == HXI_SCAN_CMD)
            is_scan_finished_ = true;

        // check for end of video image
        if (Eventer->cmd == HXI_SCAN_CMD)
            is_new_image_ = false;
    }

    // Analyse Event In Progress
    else if (Eventer->mode == HXI_PROGRESS_EVENT)
    {
        if (Eventer->cmd == HXI_SCAN_CMD || Eventer->cmd == HXI_RESUME_CMD)
        {
            is_scan_finished_ = false;
            std::string percentage = std::to_string(Eventer->value[0] * 100.0);
            string_result = "scanning progress: " + percentage + " %";
        }
    }

    // Analyse Video Event
    else if (Eventer->mode == HXI_VIDEO_EVENT)
    {
        is_new_image_ = true;
        string_result = "Video imagen";
    }

    return string_result;
}

bool EventAnalyser::isError(HxiEventT* Eventer)
{
    if (Eventer->error != 0)
        return true;
    return false;
}

std::string EventAnalyser::readEventValues(float value[2])
{
    std::string val0 = std::to_string(value[0]);
    std::string val1 = std::to_string(value[1]);

    return "[" + val0 + "," + val1 + "]";
}

std::string EventAnalyser::readEventMode(int mode)
{
    switch (mode)
    {
      case HXI_NO_EVENT:
          return "No event";
      case HXI_DONE_EVENT:
          return "Done event";
      case HXI_START_EVENT:
          return "Start event";
      case HXI_PROGRESS_EVENT:
          return "Progress event";
      case HXI_VIDEO_EVENT:
          return "Video event";
      default:
          return "";
    }
}

std::string EventAnalyser::readEventCmd(int cmd)
{
    switch (cmd)
    {
      case HXI_NO_CMD:
          return "No cmd";
      case HXI_CONNECT_CMD:
          return "Connect cmd";
      case HXI_DISCONNECT_CMD:
          return "Disconnect cmd";
      case HXI_START_VIDEO_CMD:
          return "Video start cmd";
      case HXI_STOP_VIDEO_CMD:
          return "Video stop cmd";
      case HXI_ZOOM_VIDEO_CMD:
          return "Zoom video cmd";
      case HXI_POINT_VIDEO_CMD:
          return "Point video cmd";
      case HXI_POINT_LASER_CMD:
          return "Point laser cmd";
      case HXI_GET_RANGE_CMD:
          return "Get Range cmd";
      case HXI_SCAN_CMD:
          return "Scan cmd";
      case HXI_IMAGE_CMD:
          return "Image cmd";
      case HXI_PIX_TO_ANG_CMD:
          return "Pixel to angle cmd";
      case HXI_GET_ANG_CMD:
          return "Get Angle cmd";
      case HXI_SET_TILT_CMD:
          return "Set Tilt cmd";
      case HXI_PAUSE_CMD:
          return "Pause cmd";
      case HXI_RESUME_CMD:
          return "Resume cmd";
      case HXI_CANCEL_CMD:
          return "Cancel cmd";
      case HXI_UNKNOWN_CMD:
          return "Unknown cmd";
      default:
          return "";
    }
}

std::string EventAnalyser::readDoneEventCmdAndValues(int cmd, float value[2])
{
    switch (cmd)
    {
      case HXI_CONNECT_CMD:
          return "Connected";
      case HXI_DISCONNECT_CMD:
          return "Disconnected";
      case HXI_GET_ANG_CMD:
          return "Current angles: " + readEventValues(value);
      case HXI_GET_RANGE_CMD:
          return "Scan range: " + readEventValues(value) + " m";
      case HXI_SCAN_CMD:
          return "Scan Finished!";
      case HXI_PAUSE_CMD:
          return "Paused";
      case HXI_RESUME_CMD:
          return "Resumed";
      case HXI_CANCEL_CMD:
          return "Canceled";
      case HXI_SET_TILT_CMD:
          return "Tilt changed";
      case HXI_PIX_TO_ANG_CMD:
          return "Pixel angles: " + readEventValues(value);
      case HXI_START_VIDEO_CMD:
          return "Video started. Publishing on topic: /image";
      case HXI_STOP_VIDEO_CMD:
          return "Video stoped";
      default:
          return "";
    }
}

void EventAnalyser::assemblePublishMsg(diagnostic_msgs::DiagnosticStatus* event_msg, HxiEventT* Eventer)
{
    event_msg->level = isError(Eventer)? 2:0; // level values: OK=0, WARN=1, ERROR=2, STALE=3
    event_msg->name = readEventMode(Eventer->mode);
    event_msg->message = readEventCmd(Eventer->cmd);
    event_msg->hardware_id = DEVICE_NAME_;
    event_msg->values[0].value = Eventer->value[0];
    event_msg->values[1].value = Eventer->value[1];
}

// Initialization
bool EventAnalyser::is_new_image_ = false;
bool EventAnalyser::is_scan_finished_ = false;
const std::string EventAnalyser::DEVICE_NAME_ = "LeicaC5";
