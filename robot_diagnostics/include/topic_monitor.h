#ifndef TOPIC_MONITOR_H
#define TOPIC_MONITOR_H
#include "topic_statistics.h"

class TopicMonitor
{
public:

    /* Constructor for the Ros class */ 
    TopicMonitor(std::unique_ptr <TopicStatistics> stat);

     /* Destructor for the Ros class */
    ~TopicMonitor();

    void topicMonitoring();

private:
    /*ROS Node Handler*/
     std::unique_ptr <TopicStatistics> m_statistics;
    //Statistics* m_statistics;

};


#endif