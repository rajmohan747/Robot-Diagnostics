#ifndef TOPIC_STATISTICS_H
#define TOPIC_STATISTICS_H
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <bits/stdc++.h> 
#include <mutex>
using namespace std;
  /**
   * @class TrajectoryController
   * @brief A controller that follows the trajectory provided by a planner.
   */


struct GenericSubsriber
{
    public:
        static std::string md5;
        static std::string data_type;
        static std::string const &__s_getMD5Sum() { return md5; }
        static std::string const &__s_getDataType() { return data_type; }
        void deserialize(void *) {}
};
class TopicStatistics
{
public:
        /**
     * @brief  Constructor for the TrajectoryController
     */



    TopicStatistics(ros::NodeHandle &nh,std::string topicName,double topicFrequency);

        /**
        * @brief  Destructor for the TrajectoryController
        */
    ~TopicStatistics();

    ros::NodeHandle nh;

     

    //void robot_cb(const geometry_msgs::Pose& robot_msg);
    void genericMessageCallback(const GenericSubsriber &data);


private:

    void timerCallback(const ros::TimerEvent &e);
    uint64_t millis(); 
    geometry_msgs::Pose m_robotPose;

        /**
     * @brief ROS Subscribers
     */


    //ros::Subscriber robotPose;
    ros::Subscriber universalSub;
    ros::Timer topicStatusTimer; 
    int m_currentSize,m_lastSize;
    uint64_t m_currentTime,m_lastTime,m_deltaTime;
    uint64_t m_startTime,m_endTime,m_diffTime;
    double m_averageFrequency = 0.0;
    double m_expectedFrequency;
    double m_minAcceptableFrequencyFactor;

    bool m_setup = false;
    
    std::vector<double> m_timeDifferences;
    std::string m_topic;
    std::mutex m_mutex;
};
#endif
