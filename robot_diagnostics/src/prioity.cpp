#include <iostream>
#include <ros/ros.h>
#include "xmlrpcpp/XmlRpcValue.h"
#include <queue> 


#include <vector>
#include <memory>
#include <unordered_map>
#include "monitoring_msgs/MonitoringArray.h"
#include "monitoring_msgs/KeyValue.h"
    // //std::priority_queue <monitoring_msgs::KeyValue> gquiz;
    // struct CompareHeight 
    // { 
    //     bool operator()(monitoring_msgs::KeyValue const& p1, monitoring_msgs::KeyValue const& p2) 
    //     { 
    //         return true;//p1.errorlevel < p2.errorlevel; 
    //     } 
    // }; 

    // struct Person 
    // { 
    //     std::string value; 
    //     float errorlevel; 
  
    // // this will used to initialize the variables 
    //     // of the structure 
    //     Person(std::string value, float errorlevel) 
    //         : value(value), errorlevel(errorlevel) 
    //     { 
    //     } 
    // };


struct Person { 
  
    int age; 
  
    float height; 
  
    // this will used to initialize the variables 
    // of the structure 
    Person(int age, float height) 
        : age(age), height(height) 
    { 
    } 
}; 
  
// this is an strucure which implements the 
// operator overlading 
struct CompareHeight { 
    bool operator()(Person const& p1, Person const& p2) 
    { 
        // return "true" if "p1" is ordered  
        // before "p2", for example: 
        return p1.height < p2.height; 
    } 
}; 

/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "priority");
    ros::NodeHandle n;
    ros::Rate rate(1);
    
    std::priority_queue<Person, std::vector<Person>, CompareHeight> Q; 

    Q.push(Person(10, 3.5));
    //boost::thread* controller_thread = new boost::thread(boost::bind(&NavigationWrapper::controlFlow, &controller));
   
    while(ros::ok())
    {
      
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}