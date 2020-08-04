#ifndef TOPIC_MONITOR_H
#define TOPIC_MONITOR_H

#include <iostream>
#include <ros/ros.h>


#include "topic_statistics.h"
#include "utilities.h"

class TopicMonitor
{
public:

    /* Constructor for the TopicMonitor */ 
    TopicMonitor();

     /* Destructor for the TopicMonitor */
    ~TopicMonitor();


private:
    ros::NodeHandle nh;
    ros::Timer topicUpdateTimer; 

    /*Member Functions*/
    void validTopicList(std::unordered_map<std::string ,double> &validTopicMap);
    //void getAllTopics();
    //bool isValidTopic(std::string &topic);
    void topicTimerCallback(const ros::TimerEvent &e);
    void topicErrorMap(); 
    
    /*Member variables*/
    std::vector<std::string> m_topicListOriginal;
    std::vector<std::shared_ptr<TopicStatistics> > topicMonitorList;
    std::unordered_map<std::string ,double> m_validTopicMap;
    std::unordered_map<std::string ,double> m_invalidTopicMap;
    std::vector<std::string> m_invalidTopicList; 
    XmlRpc::XmlRpcValue m_topicList;
    bool m_invalidTopics = false;

    double m_topicTimeOut = 10.0;
    double m_minAcceptableFrequencyFactor;
    double m_topicDataTimeOut = 1.0;

    uint64_t m_lastTime;


    std::shared_ptr<Monitor> m_topicMonitor;
    TopicParams m_topicParam; 
    XmlRpc::XmlRpcValue m_topicErrors;

};


#endif
