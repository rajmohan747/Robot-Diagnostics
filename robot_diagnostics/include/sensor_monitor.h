#ifndef SENSOR_MONITOR_H
#define SENSOR_MONITOR_H
#include "sensor_statistics.h"


class SensorMonitor
{
public:

    /* Constructor for the SensorMonitor class */ 
    SensorMonitor();

     /* Destructor for the SensorMonitor class */
    ~SensorMonitor();


private:

    ros::NodeHandle nh;

    /*Member functions*/    
    void getAllTopics();
    void validTopicList(std::vector<std::string> &validTopicList);
    bool isValidTopic(std::string &topic_name);



    /*Member variables*/
    std::vector<std::string> m_sensorTopicList;
    std::vector<std::string> m_topicListOriginal;
    std::vector<std::shared_ptr<SensorStatistics> > sensorMonitorList;


    std::shared_ptr<Monitor> m_sensorMonitor;
};


#endif