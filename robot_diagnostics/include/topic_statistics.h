#ifndef TOPIC_STATISTICS_H
#define TOPIC_STATISTICS_H
#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <bits/stdc++.h> 
#include <mutex>
#include <memory>
#include "monitoring_core/monitor.h"
#include "xmlrpcpp/XmlRpcValue.h"
#include <unordered_map>
#include <string>
/**
* @class TopicStatistics
*/


struct GenericSubsriber
{
    public:
        static std::string md5;
        static std::string data_type;
        static std::string const &__s_getMD5Sum() { return md5; }
        static std::string const &__s_getDataType() { return data_type; }
        void deserialize(void *) {}
};
class TopicStatistics
{
public:
/**
 * @brief  Constructor for the TopicStatistics
 */



    TopicStatistics(ros::NodeHandle &nh,std::string topicName,double topicFrequency,std::shared_ptr<Monitor> monitor);

    /**
    * @brief  Destructor for the TopicStatistics
    */
    ~TopicStatistics();
    


private:

    ros::NodeHandle nh;

    /**
     * @brief ROS Subscribers
    */


    ros::Subscriber universalSub;
    ros::Timer topicStatusTimer; 

    /*Member functions*/
    void genericMessageCallback(const GenericSubsriber &data);
    void timerCallback(const ros::TimerEvent &e);
    void publishNoTopicInfo();
    void publishTopicDelayInfo();
    void publishTopicOkInfo();
    uint64_t millis(); 


    /*Member variables*/

    int m_currentSize,m_lastSize;
    uint64_t m_currentTime,m_lastTime,m_deltaTime;
    uint64_t m_startTime,m_endTime,m_diffTime;
    double m_averageFrequency = 0.0;
    double m_timeOut = 1.0;
    double m_expectedFrequency;
    double m_minAcceptableFrequencyFactor;

    bool m_setup = false;
    bool m_topicHealth = true;
    std::vector<double> m_timeDifferences;
    std::string m_topic;
    std::mutex m_mutex;

    std::shared_ptr<Monitor> m_monitor;
};
#endif
