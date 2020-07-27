#ifndef TOPIC_MONITOR_H
#define TOPIC_MONITOR_H

#include <iostream>
#include <ros/ros.h>


#include "topic_statistics.h"


class TopicMonitor
{
public:

    /* Constructor for the TopicMonitor */ 
    TopicMonitor();

     /* Destructor for the TopicMonitor */
    ~TopicMonitor();


private:
    ros::NodeHandle nh;

    /*Member Functions*/
    void validTopicList(std::unordered_map<std::string ,double> &validTopicMap);
    void getAllTopics();
    bool isValidTopic(std::string &topic);

    /*Member variables*/
    std::vector<std::string> m_topicListOriginal;
    std::vector<std::shared_ptr<TopicStatistics> > topicMonitorList;
    std::unordered_map<std::string ,double> m_validTopicMap;
    XmlRpc::XmlRpcValue m_topicList;


    std::shared_ptr<Monitor> m_topicMonitor;

};


#endif
