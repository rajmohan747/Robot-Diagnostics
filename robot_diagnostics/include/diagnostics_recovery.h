#ifndef DIAGNOSTICS_RECOVERY_H
#define DIAGNOSTICS_RECOVERY_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <mutex>
#include <unistd.h>
#include <memory>
#include <thread>
#include <algorithm>
#include <future>
#include "monitoring_msgs/KeyValue.h"
#include "monitoring_msgs/KeyValues.h"
/*
#include <memory>
#include <mutex>
#include <unordered_map>
#include "monitoring_msgs/MonitoringArray.h"

*/


struct Recovery
{
    std::string node;
    std::string package;
    int type;
};


std::mutex mtx;
class SystemCommand
{
    private:
        //std::mutex m_mutex;
    public:
        
        void sendCommand(std::string command)
        {
            //std::unique_lock<std::mutex> commandLock(mtx);
            ROS_ERROR("Thread called ..sendCommand : %s",command.c_str());
            //int output = system(command.c_str());
        }
};

class DiagnosticsRecovery
{
public:

    /* Constructor for the DiagnosticsRecovery class */ 
    DiagnosticsRecovery();

     /* Destructor for the DiagnosticsRecovery class */
    ~DiagnosticsRecovery();

   
private:

    ros::NodeHandle nh;

    /*Subscribers*/
    ros::Subscriber criticalSub;
    ros::Subscriber nonCriticalSub;

    /*Publishers*/


    /*Timer*/

    
     /*Member functions*/
   
    void criticalMessageCallback(const monitoring_msgs::KeyValuesConstPtr &msg);

    void criticalErrorClassification(std::string keyValue);
    
    void recoveryProcedure(std::string nodeName);

    void systemCommand(std::string command);
    /*Member variables*/

    int m_index = 0;

    XmlRpc::XmlRpcValue m_diagnosticsRecovery;
    std::vector<Recovery> m_recoveryParams;
    std::mutex m_mutex;
    std::vector<std::thread> m_thread;
    std::vector<std::future<void>> m_futures;
    std::vector<SystemCommand> m_systemCommand;

};


#endif