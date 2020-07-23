#include "system_monitor.h"


/**
* @brief  Constructor for the NodeMonitor
*/
SystemMonitor::SystemMonitor(std::unique_ptr <SystemStatistics> stat):m_statistics(std::move(stat))
{
    ROS_INFO("System monitor constructor initialized");
}

/**
* @brief  Destructor for the NodeMonitor
*/

SystemMonitor::~SystemMonitor()
{

}

/**
* @brief  Collects the updated data related to node monitoring
*/
void SystemMonitor::systemMonitoring()
{
    m_statistics->updateSystemStatistics();
}


/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "system_monitoring");
    std::unique_ptr<SystemStatistics> statisticsPtr(new SystemStatistics);
    SystemMonitor system_monitor(std::move(statisticsPtr));
    //NodeMonitor node_monitor(new Statistics());
    ros::Rate rate(1);

    //boost::thread* controller_thread = new boost::thread(boost::bind(&NavigationWrapper::controlFlow, &controller));

    while(ros::ok())
    {
        system_monitor.systemMonitoring();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}