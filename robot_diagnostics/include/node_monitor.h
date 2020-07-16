#ifndef NODE_MONITOR_H
#define NODE_MONITOR_H
#include "node_statistics.h"

class NodeMonitor
{
public:

    /* Constructor for the Ros class */ 
    NodeMonitor(std::unique_ptr <NodeStatistics> stat);

     /* Destructor for the Ros class */
    ~NodeMonitor();

    void nodeMonitoring();

private:
    /*ROS Node Handler*/
     std::unique_ptr <NodeStatistics> m_statistics;
    //Statistics* m_statistics;

};


#endif