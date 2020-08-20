
#include "diagnostics_recovery.h"


DiagnosticsRecovery::DiagnosticsRecovery()
{
    ROS_INFO("Constructor of DiagnosticsRecovery called");
    /*Parameters from parameter server*/


    /*Subscribers*/
    criticalSub = nh.subscribe("/critical_errors", 1, &DiagnosticsRecovery::criticalMessageCallback, this);

    /*Publishers*/
    
    
    /*Timer*/
  
  

}


DiagnosticsRecovery::~DiagnosticsRecovery()
{

}


void DiagnosticsRecovery::criticalMessageCallback(const monitoring_msgs::KeyValuesConstPtr &msg)
{
    ROS_INFO("Message size : %d",msg->keyvalues.size());
    for(int i =0; i < msg->keyvalues.size();i++)
    {
       
        std::string keyValue = msg->keyvalues[i].key;
        criticalErrorClassification(keyValue);
        
    }
}


void DiagnosticsRecovery::criticalErrorClassification(std::string keyValue)
{
    std::string nodeUnavailable = "node_unavailable";
    if (keyValue.find(nodeUnavailable) != std::string::npos)
    {
        std::size_t charPosition = keyValue.find(":");
        if (charPosition!=std::string::npos)
        {
            std::string key   = keyValue.substr(1,charPosition);
            std::string value = keyValue.substr(charPosition+1);
            ROS_WARN("Extracted key : %s with value : %s ",key.c_str(),value.c_str());
        }         
    }   
}
/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "diagnostics_recovery");
    ROS_INFO("Main of DiagnosticsRecovery called");
    /* Create an object to DiagnosticsRecovery */
    DiagnosticsRecovery diagnosticsRecovery;

    ros::NodeHandle n;
    ros::Rate rate(1);
    
    while(ros::ok())
    {
        //diagnosticsAggregator.errorCategorization();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
