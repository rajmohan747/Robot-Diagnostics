#ifndef TOPIC_STATISTICS_H
#define TOPIC_STATISTICS_H

#include <iostream>
#include <ros/ros.h>



// Enums
enum FilterType
{
    DEFAULT,
    ADD,
    REMOVE
};

class TopicStatistics
{
public:

    /* Constructor for the Ros class */ 
    TopicStatistics();

     /* Destructor for the Ros class */
    ~TopicStatistics();

    void updateTopicStatistics();

private:
    ros::NodeHandle nh;

    /*Member Functions*/
    void validTopicList(const std::vector<std::string> &initialTopicList,std::vector<std::string> &validTopicList);
    void getAllTopics();
    void applyTopicFilter();
    bool isValidTopic(std::string &topic);
    int m_topicFilterType = 0;
    std::vector<std::string> m_validTopicList,m_topicListOriginal,m_topicListCopy;


};


#endif