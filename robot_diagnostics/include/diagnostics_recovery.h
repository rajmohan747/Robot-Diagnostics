#ifndef DIAGNOSTICS_RECOVERY_H
#define DIAGNOSTICS_RECOVERY_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include "monitoring_msgs/KeyValue.h"
#include "monitoring_msgs/KeyValues.h"
/*
#include <memory>
#include <mutex>
#include <unordered_map>
#include "monitoring_msgs/MonitoringArray.h"

*/






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
    

    /*Member variables*/


};


#endif