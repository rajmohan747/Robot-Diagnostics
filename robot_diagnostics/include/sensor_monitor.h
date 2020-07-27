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
    void validTopicList(std::unordered_map<std::string ,std::string> &validTopicMap);
    bool isValidTopic(std::string &topic_name);



    /*Member variables*/
    std::vector<std::string> m_topicListOriginal;
    std::unordered_map<std::string ,std::string> m_validTopicMap;
    XmlRpc::XmlRpcValue m_topicList;
    std::vector<std::shared_ptr<SensorStatistics> > monitor_list_;

    //Monitor *monitor_;
    std::shared_ptr<Monitor> m_topicMonitor;
};


#endif