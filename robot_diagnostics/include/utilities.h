#ifndef UTILITIES_H
#define UTILITIES_H

#include <iostream>
#include <chrono>


  /**
   * @Namespace math computation
   */

namespace Utilities
{

    const std::string kProcDirectory{"/proc/"};
    const std::string kStatusFilename{"/status"};
    const std::string kUptimeFilename{"/uptime"};
    const std::string kStatFilename{"/stat"};
    const std::string kMeminfoFilename{"/meminfo"};
    
    enum NodeFilter
    {
        DEFAULT,
        ADD,
        REMOVE
    };


    enum CPUStates 
    {
        kUser_ = 0,
        kNice_,
        kSystem_,
        kIdle_,
        kIOwait_,
        kIRQ_,
        kSoftIRQ_,
        kSteal_,
        kGuest_,
        kGuestNice_
    };

/* @brief  Access the system time using chrono library
* @return time in milliseconds
*/
    template <typename T>
    T millis() 
    {
        T ms =std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        return ms;
    }
    
    /**
    * @brief checks whehter the given node name is currently registered with ros master
    * @returns true if the node is available,else false;
    */
    
    template <typename T,typename U>
    T isValidNode(U &node_name)
    {
        std::vector<std::string> currentNodeList;
        ros::master::getNodes(currentNodeList);
        for(auto node : currentNodeList)
        {
            if(node == node_name)
            {
            return true;
            }
        }
        return false;
    }  

    /**
    * @brief  Verifying whether the given topic is registered with the  ROS master
    */
    template <typename T,typename U>
    T isValidTopic(U &topic_name,const std::vector<U> &topicListOriginal)
    {

        for(auto topic : topicListOriginal)
        {
            if(topic == topic_name)
            {
            return true;
            }
        }
        return false;

    }


    /**
    * @brief  Getting all the topics registered with the ROS master
    */
    template <typename T,typename U>
    T getAllTopics(std::vector<U> &topicListOriginal)
    {
        topicListOriginal.clear();
        topicListOriginal.resize(0);
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);
        
        for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) 
        {
            topicListOriginal.push_back((*it).name);
        }
    }

};

#endif
