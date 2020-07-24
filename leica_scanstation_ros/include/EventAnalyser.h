// EventAnalyser.h
#pragma once
#ifndef _EVENTANALISER_H
#define _EVENTANALISER_H

#include <afx.h>
#include <leica_scanstation_msgs/EventerInfo.h>
#include "HxiDefinitions.h"

#endif 

/** @brief Pointer to function for event handler. */
typedef int (*notify)(HxiEventT*);

typedef HxiEventT  LeicaEvent;
typedef HxiEventT* LeicaEventPtr;

/**
 * @brief The Leica Scanstation C5 SDK throws events containing information about the current process. 
 *         \n For example, an event indicating DONE when connection between Leica and PC is required. 
 *         \n To be able to handle this events, the programmer should create an Event Handler and
 *            combine it with this class to understand the information received.
 * 
 */
class EventAnalyser {

public:

    /**
     * @brief Construct a new Event Analyser object
     * 
     */
    explicit EventAnalyser() {};

    /**
     * @brief Destroy the Event Analyser object
     * 
     */
    virtual ~EventAnalyser() {}


    /** @brief Flag to indicate a new image is available */
    static bool _is_new_image;

    /** @brief Flag to indicate the scan process is finished */
    static bool _is_scan_finished;


    /**
     * @brief Get a string specifying the meaning of the received event.
     *        \n Set also flags to know system state.
     * 
     * @param Eventer 
     * @return std::string 
     */
    static std::string getStringResult(HxiEventT* Eventer);

    /**
     * @brief Check if the event received indicates error.
     * 
     * @param Eventer 
     * @return true 
     * @return false 
     */
    static bool isError(HxiEventT* Eventer);

    /**
     * @brief Get a string with the values received next to the event.
     * 
     * @param value 
     * @return std::string 
     */
    static std::string readEventValues(float value[2]);

    /**
     * @brief Get a string with the current mode of the device, such as started or done process.
     * 
     * @param mode 
     * @return std::string 
     */
    static std::string readEventMode(int mode);

    /**
     * @brief Get a string with the process command sent by the scanner, such as connecting, scanning...
     * 
     * @param cmd 
     * @return std::string 
     */
    static std::string readEventCmd(int cmd);

    /**
     * @brief Get a string with the process finished and it's associated values sent by the scanner.
     * 
     * @param cmd 
     * @param value 
     * @return std::string 
     */
    static std::string readDoneEventCmdAndValues(int cmd, float value[2]);

    /**
     * @brief In order to publish the event information, this method creates an EventerInfo ROS message.
     * 
     * @param event_msg 
     * @param Eventer 
     */
    static void assemblePublishMsg(leica_scanstation_msgs::EventerInfo *event_msg,
                                    HxiEventT* Eventer);

    /**
     * @brief Asign Event Handler with a function pointer of type notify.
     * 
     * @param event_handler 
     */
    static void setEventHandler(notify event_handler);


    static int ScannerEventHandler(HxiEventT* Eventer);
};