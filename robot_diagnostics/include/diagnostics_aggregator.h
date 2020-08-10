#ifndef DIAGNOSTICS_AGGREGATOR_H
#define DIAGNOSTICS_AGGREGATOR_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <memory>
#include <mutex>
#include <unordered_map>
#include "monitoring_msgs/MonitoringArray.h"
#include "monitoring_msgs/KeyValue.h"
#include "monitoring_msgs/KeyValues.h"
#include "analyser.h"
#include <queue> 
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

#define CRITICAL_ERROR 0.8
#define VITAL_ERROR 0.6
#define NON_CRITICAL_ERROR 0.3

using namespace message_filters;


struct CompareError 
{ 
    bool operator()(monitoring_msgs::KeyValue const& p1, monitoring_msgs::KeyValue const& p2) 
    { 
        return p1.errorlevel < p2.errorlevel; 
    } 
}; 


class DiagnosticsAggregator
{
public:

    /* Constructor for the DiagnosticsAggregrator class */ 
    DiagnosticsAggregator();

     /* Destructor for the DiagnosticsAggregrator class */
    ~DiagnosticsAggregator();
    
    void errorCategorization();


   
private:

    ros::NodeHandle nh;

    /*Subscribers*/
    message_filters::Subscriber <monitoring_msgs::MonitoringArray> nodeSubscriber;
    message_filters::Subscriber <monitoring_msgs::MonitoringArray> topicSubscriber;
    message_filters::Subscriber <monitoring_msgs::MonitoringArray> systemSubscriber;
    message_filters::Subscriber <monitoring_msgs::MonitoringArray> sensorSubscriber;

    /*Publishers*/
    ros::Publisher criticalPub;
    ros::Publisher nonCriticalPub;

    /*Timer*/
    ros::Timer clearQueueTimer; 
    
    /*Approximate Time Sychronizer*/
    typedef sync_policies::ApproximateTime<monitoring_msgs::MonitoringArray, 
    monitoring_msgs::MonitoringArray,monitoring_msgs::MonitoringArray,
    monitoring_msgs::MonitoringArray> m_synchronizerPolicy;

    Synchronizer<m_synchronizerPolicy> m_sychronizer;

     /*Member functions*/
    void errorCallback(const monitoring_msgs::MonitoringArrayConstPtr& nodeError,
    const monitoring_msgs::MonitoringArrayConstPtr& topicError,
    const monitoring_msgs::MonitoringArrayConstPtr& systemError,
    const monitoring_msgs::MonitoringArrayConstPtr& sensorError);

   
    

    void clearQueueTimerCallback(const ros::TimerEvent &e);
    void errorAnalysis(std::vector<std::string> &errorKey,std::unordered_map<std::string,int> &nodeErrorList,monitoring_msgs::KeyValue &nodeErrors,int maxErrorOccurence);

    bool errorMessageUpdate(const std::vector<monitoring_msgs::KeyValue> &errors,const std::vector<monitoring_msgs::KeyValue> &errorsLast);
    void resetParameters();

    /*Member variables*/

    int m_nodeMaxErrorOccurences,m_topicMaxErrorOccurences,m_systemMaxErrorOccurences,m_sensorMaxErrorOccurences;
    std::vector<monitoring_msgs::KeyValue> m_nodeErrors,m_topicErrors,m_systemErrors,m_sensorErrors; 
    std::vector<monitoring_msgs::KeyValue> m_nodeErrorsLast,m_topicErrorsLast,m_systemErrorsLast,m_sensorErrorsLast; 

    std::unordered_map<std::string,int> m_nodeErrorCountList,m_topicErrorCountList,m_systemErrorCountList,m_sensorErrorCountList;
    

    /*Applicable for critical errors > m_nodeMaxErrorOccurences*/
    std::vector<std::string> m_nodeErrorKey,m_topicErrorKey,m_systemErrorKey,m_sensorErrorKey;
    
    std::mutex m_mutex;


    /*This is an strucure which implements the  operator overloading */ 

    std::priority_queue<monitoring_msgs::KeyValue, std::vector<monitoring_msgs::KeyValue>, CompareError> m_errorQueue;

    monitoring_msgs::KeyValues m_criticalErrors,m_nonCriticalErrors;

};


#endif