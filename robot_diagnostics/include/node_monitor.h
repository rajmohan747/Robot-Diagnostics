#ifndef NODE_MONITOR_H
#define NODE_MONITOR_H


#include "node_statistics.h"



class NodeMonitor
{
public:

    /* Constructor for the NodeMonitor class */ 
    NodeMonitor();

     /* Destructor for the NodeMonitor class */
    ~NodeMonitor();


private:

    ros::NodeHandle nh;

    ros::Timer nodeUpdateTimer;

    /*Member functions*/

    bool isValidNode(std::string &nodeName);;
    void updateValidNodeList(std::vector<std::string> &validNodeList);
    void applyNodeFilter(std::vector<std::string> &nodeListFiltered);
    void nodeTimerCallback(const ros::TimerEvent &e);
    uint64_t millis();
    /*Member variables*/

    int m_nodeFilterType = 0;
    double m_nodeTimeOut = 1.0;
    bool m_invalidNodes =false;
    uint64_t m_lastTime;
    std::vector<std::shared_ptr<NodeStatistics> >m_nodeList;
    std::vector<std::string> m_initialNodeList,m_validNodeList,m_invalidNodeList;
    std::vector<std::string> m_nodeListOriginal,m_nodeListFiltered;

    //Monitor *monitor_;
    std::shared_ptr<Monitor> m_nodeMonitor;

};


#endif