#ifndef TOPIC_MONITOR_H
#define TOPIC_MONITOR_H

#include <iostream>
#include <ros/ros.h>
#include "utils.h"

#include "topic_statistics.h"
#include "xmlrpcpp/XmlRpcValue.h"
#include <unordered_map>


class TopicMonitor
{
public:

    /* Constructor for the Ros class */ 
    TopicMonitor();

     /* Destructor for the Ros class */
    ~TopicMonitor();


private:
    ros::NodeHandle nh;

    /*Member Functions*/
    void validTopicList(std::unordered_map<std::string ,double> &validTopicMap);
    void getAllTopics();
    bool isValidTopic(std::string &topic);

    std::vector<std::string> m_validTopicList,m_topicListOriginal,m_topicListCopy;
    std::vector<shared_ptr<TopicStatistics> > monitor_list_;
    std::unordered_map<std::string ,double> m_validTopicMap;
    XmlRpc::XmlRpcValue m_topicList;

    // std::vector<FrequencyStatistics *> monitor_list_;

};


#endif