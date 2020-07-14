#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H
#include "system_statistics.h"

class SystemMonitor
{
public:

    /* Constructor for the Ros class */ 
    SystemMonitor(std::unique_ptr <SystemStatistics> stat);

     /* Destructor for the Ros class */
    ~SystemMonitor();

    void systemMonitoring();

private:
    /*ROS Node Handler*/
     std::unique_ptr <SystemStatistics> m_statistics;
    //Statistics* m_statistics;

};


#endif