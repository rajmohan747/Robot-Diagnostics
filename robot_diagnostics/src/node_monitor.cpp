#include "node_monitor.h"


/**
* @brief  Constructor for the NodeMonitor
*/
NodeMonitor::NodeMonitor(std::unique_ptr <Statistics> stat):m_statistics(std::move(stat))
{
    ROS_INFO("Node monitor constructor initialized");
}

/**
* @brief  Destructor for the NodeMonitor
*/

NodeMonitor::~NodeMonitor()
{

}

/**
* @brief  Collects the updated data related to node monitoring
*/
void NodeMonitor::nodeMonitoring()
{
    m_statistics->updateStatistics();
}


/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_monitoring");
    std::unique_ptr<Statistics> statisticsPtr(new Statistics);
    NodeMonitor node_monitor(std::move(statisticsPtr));
    //NodeMonitor node_monitor(new Statistics());
    ros::Rate rate(1);

    //boost::thread* controller_thread = new boost::thread(boost::bind(&NavigationWrapper::controlFlow, &controller));

    while(ros::ok())
    {
        node_monitor.nodeMonitoring();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}