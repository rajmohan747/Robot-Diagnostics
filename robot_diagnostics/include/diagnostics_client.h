#ifndef DIAGNOSTICS_CLIENT_H
#define DIAGNOSTICS_CLIENT_H

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

struct CompareError 
{ 
    bool operator()(monitoring_msgs::KeyValue const& p1, monitoring_msgs::KeyValue const& p2) 
    { 
        return p1.errorlevel < p2.errorlevel; 
    } 
}; 


class DiagnosticsClient
{
public:

    /* Constructor for the DiagnosticsClient class */ 
    DiagnosticsClient();

     /* Destructor for the DiagnosticsClient class */
    ~DiagnosticsClient();
    
    void errorCategorization();


   
private:

    ros::NodeHandle nh;

    ros::Subscriber nodeSubscriber;
    ros::Subscriber topicSubscriber;    
    ros::Subscriber systemSubscriber;
    ros::Subscriber sensorSubscriber;

    ros::Publisher criticalPub;
    ros::Publisher nonCriticalPub;

    ros::Timer clearQueueTimer; 

    /*Member functions*/
    void NodeDiagnosticsCallback(const monitoring_msgs::MonitoringArrayConstPtr &error);
    void TopicDiagnosticsCallback(const monitoring_msgs::MonitoringArrayConstPtr &error);
    void SystemDiagnosticsCallback(const monitoring_msgs::MonitoringArrayConstPtr &error);
    void SensorDiagnosticsCallback(const monitoring_msgs::MonitoringArrayConstPtr &error);

    void clearQueueTimerCallback(const ros::TimerEvent &e);
    void errorAnalysis(std::vector<std::string> &errorKey,std::unordered_map<std::string,int> &nodeErrorList,monitoring_msgs::KeyValue &nodeErrors,int maxErrorOccurence);
    


    /*Member variables*/

    int m_nodeMaxErrorOccurences,m_topicMaxErrorOccurences,m_systemMaxErrorOccurences,m_sensorMaxErrorOccurences;
    std::vector<monitoring_msgs::KeyValue> m_nodeErrors,m_topicErrors,m_systemErrors,m_sensorErrors; 
    std::unordered_map<std::string,int> m_nodeErrorCountList,m_topicErrorCountList,m_systemErrorCountList,m_sensorErrorCountList;
    

    /*Applicable for critical errors > m_nodeMaxErrorOccurences*/
    std::vector<std::string> m_nodeErrorKey,m_topicErrorKey,m_systemErrorKey,m_sensorErrorKey;

    //std::shared_ptr<Analyser> m_analyzer;
    std::mutex m_mutex;

// this is an strucure which implements the 
// operator overlading 

    std::priority_queue<monitoring_msgs::KeyValue, std::vector<monitoring_msgs::KeyValue>, CompareError> m_errorQueue;
    //std::priority_queue<monitoring_msgs::KeyValue, std::vector<monitoring_msgs::KeyValue>, CompareError> errorQueue;

    monitoring_msgs::KeyValues m_criticalErrors,m_nonCriticalErrors;

};


#endif