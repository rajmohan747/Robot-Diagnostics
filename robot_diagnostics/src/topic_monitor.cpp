#include "topic_monitor.h"


/**
* @brief  Constructor for the NodeMonitor
*/
TopicMonitor::TopicMonitor(std::unique_ptr <TopicStatistics> stat):m_statistics(std::move(stat))
{
    ROS_INFO("System monitor constructor initialized");
}

/**
* @brief  Destructor for the NodeMonitor
*/

TopicMonitor::~TopicMonitor()
{

}

/**
* @brief  Collects the updated data related to node monitoring
*/
void TopicMonitor::topicMonitoring()
{
    m_statistics->updateTopicStatistics();
}


/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "topic_monitoring");
    std::unique_ptr<TopicStatistics> statisticsPtr(new TopicStatistics);
    TopicMonitor topic_monitor(std::move(statisticsPtr));
    //NodeMonitor node_monitor(new Statistics());
    ros::Rate rate(5);

    //boost::thread* controller_thread = new boost::thread(boost::bind(&NavigationWrapper::controlFlow, &controller));

    while(ros::ok())
    {
        topic_monitor.topicMonitoring();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}