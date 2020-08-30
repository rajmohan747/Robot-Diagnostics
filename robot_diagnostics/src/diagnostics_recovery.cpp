
#include "diagnostics_recovery.h"


DiagnosticsRecovery::DiagnosticsRecovery()
{
    ROS_INFO("Constructor of DiagnosticsRecovery called");
    /*Parameters from parameter server*/
    nh.getParam("/diagnosticsRecovery", m_diagnosticsRecovery);

    /*Subscribers*/
    criticalSub = nh.subscribe("/critical_errors", 1, &DiagnosticsRecovery::criticalMessageCallback, this);

    /*Publishers*/
    
    
    /*Timer*/
    if (m_diagnosticsRecovery.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        for(int i=0; i < m_diagnosticsRecovery.size(); i++)
        {   Recovery recoveryParam;
            XmlRpc::XmlRpcValue errorObject = m_diagnosticsRecovery[i];
            //systemErrorMap[errorObject["key"]] = errorObject["error_level"];
            recoveryParam.node     = std::string(errorObject["node"]);
            recoveryParam.package  = std::string(errorObject["package"]);
            recoveryParam.type     = errorObject["type"];

            m_recoveryParams.push_back(recoveryParam);
            std::cout << errorObject["node"] << " : " << errorObject["package"] << std::endl;
        }
    }  
  

}


DiagnosticsRecovery::~DiagnosticsRecovery()
{

}


void DiagnosticsRecovery::criticalMessageCallback(const monitoring_msgs::KeyValuesConstPtr &msg)
{
    //ROS_INFO("Message size : %d",msg->keyvalues.size());
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
            std::string key   = keyValue.substr(1,charPosition-1);
            std::string value = keyValue.substr(charPosition+1);
            recoveryProcedure(key);
            ROS_WARN("Extracted key : %s with value : %s ",key.c_str(),value.c_str());
        }         
    }   
}


void DiagnosticsRecovery::recoveryProcedure(std::string nodeName)
{
   // std::unique_lock<std::mutex> recoveryLock(m_mutex);
    std::string command;
    for(int i=0; i < m_recoveryParams.size();i++)
    {
        if(nodeName == m_recoveryParams[i].node)
        {
            if( m_recoveryParams[i].type == 1)
            {
                command = "rosrun " + m_recoveryParams[i].package +" "+ m_recoveryParams[i].node;
            }
            else
            {
                command = "roslaunch " + m_recoveryParams[i].package +" "+ m_recoveryParams[i].node+"launch";
            }
            
            
            //std::thread t1= std::thread(&DiagnosticsRecovery::systemCommand,this,command);    
            //m_thread.emplace_back(std::move(t1));

            auto ftr= std::async(std::launch::async, &SystemCommand::sendCommand,m_systemCommand[m_index], command);
            //ROS_INFO("Thread sizee : %d",m_futures.size() );
            m_futures.push_back(std::move(ftr));
            m_index = m_index + 1;
            ROS_ERROR("Command :%d ... %s",m_index,command.c_str());
        }
    }
}


void DiagnosticsRecovery::systemCommand(std::string command)
{
    std::unique_lock<std::mutex> commandLock(m_mutex);
    ROS_ERROR("Thread called ..systemCommand : %s",command.c_str());
    int output = system(command.c_str());
   // sleep(5);
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
    

    //  std::for_each(m_thread.begin(),m_thread.end(),[](std::thread &t){t.join();});
    while(ros::ok())
    {
        //diagnosticsAggregator.errorCategorization();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
