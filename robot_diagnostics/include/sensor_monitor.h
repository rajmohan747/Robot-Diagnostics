#ifndef SENSOR_MONITOR_H
#define SENSOR_MONITOR_H
#include "sensor_statistics.h"
#include "utilities.h"

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
    //void getAllTopics();
    void validTopicList(std::vector<std::string> &validTopicList);
    //bool isValidTopic(std::string &topic_name);
    void timerCallback(const ros::TimerEvent &e);


    /*Member variables*/
    std::vector<std::string> m_sensorTopicList;
    std::vector<std::string> m_topicListOriginal;
    std::vector<std::string> m_invalidTopicList; 
    std::vector<std::shared_ptr<SensorStatistics> > sensorMonitorList;

    ros::Timer topicStatusTimer; 


    std::shared_ptr<Monitor> m_sensorMonitor;
    bool m_invalidTopics = false;
};


#endif