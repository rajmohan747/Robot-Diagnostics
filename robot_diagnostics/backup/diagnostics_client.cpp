
#include "diagnostics_client.h"


DiagnosticsClient::DiagnosticsClient()
{


    nh.getParam("/nodeMaxErrorOccurences", m_nodeMaxErrorOccurences);
    nh.getParam("/topicMaxErrorOccurences", m_topicMaxErrorOccurences);
    nh.getParam("/systemMaxErrorOccurences", m_systemMaxErrorOccurences);
    nh.getParam("/sensorMaxErrorOccurences", m_sensorMaxErrorOccurences);

    nodeSubscriber   = nh.subscribe("/node_monitoring/monitoring", 1, &DiagnosticsClient::NodeDiagnosticsCallback, this);
    topicSubscriber  = nh.subscribe("/topic_monitoring/monitoring", 1, &DiagnosticsClient::TopicDiagnosticsCallback, this);
    systemSubscriber = nh.subscribe("/system_monitoring/monitoring", 1, &DiagnosticsClient::SystemDiagnosticsCallback, this);
    sensorSubscriber = nh.subscribe("/sensor_monitoring/monitoring", 1, &DiagnosticsClient::SensorDiagnosticsCallback, this);

    //nodeStatusTimer = nh.createTimer(ros::Duration(1.0), &DiagnosticsClient::nodeTimerCallback, this);
    //topicStatusTimer = nh.createTimer(ros::Duration(1.0), &DiagnosticsClient::topicTimerCallback, this);
    //systemStatusTimer = nh.createTimer(ros::Duration(1.0), &DiagnosticsClient::systemTimerCallback, this);
    //sensorStatusTimer = nh.createTimer(ros::Duration(1.0), &DiagnosticsClient::sensorTimerCallback, this);

    clearQueueTimer = nh.createTimer(ros::Duration(5.0), &DiagnosticsClient::clearQueueTimerCallback, this);

    m_analyzer =std::make_shared<Analyser>(nh);

}


DiagnosticsClient::~DiagnosticsClient()
{

}



void DiagnosticsClient::NodeDiagnosticsCallback(const monitoring_msgs::MonitoringArrayConstPtr &error)
{

    std::unique_lock<std::mutex> nodeLock(m_mutex);
    m_nodeErrors = error->info[0].values;



    for(int i =0; i< m_nodeErrors.size();i++)
    {
        /*In local perspective: ie within last clearTimer update,how many times a particaular error occured
        To be precise,to avoid duplication of same error pushed back for publishing*/
        bool nodeStatus = false;
        std::vector<std::string>::iterator it;
        it = std::find (m_nodeErrorKey.begin(), m_nodeErrorKey.end(), m_nodeErrors[i].key); 
        if(it == m_nodeErrorKey.end())
        {
            nodeStatus = true;

        }
        else
        {
            //ROS_WARN("Node Duplicated");
        }



        /*In global perspective : how many times a particular error reported*/
        if(m_nodeErrorList.find(m_nodeErrors[i].key) != m_nodeErrorList.end())
        {
            int currentCount = m_nodeErrorList[m_nodeErrors[i].key];
            m_nodeErrorList[m_nodeErrors[i].key] =currentCount + 1; 
            //ROS_WARN("%s has occured %d times",m_nodeErrors[i].key.c_str(),currentCount);
        }
        else
        {
            ROS_WARN("First time node occurance  : %s",m_nodeErrors[i].key.c_str());
            m_nodeErrorList[m_nodeErrors[i].key] = 1;
        }

        if((m_nodeErrorList[m_nodeErrors[i].key] > m_nodeMaxErrorOccurences ) && (nodeStatus == true))
        {
            m_nodeErrorKey.push_back(m_nodeErrors[i].key);
            m_errorQueue.push(m_nodeErrors[i]);
            ROS_ERROR("Node error %s has crossed limits",m_nodeErrors[i].key.c_str());
            //errorCategorization(m_nodeErrors[i]);
        }
    }
    
}

void DiagnosticsClient::TopicDiagnosticsCallback(const monitoring_msgs::MonitoringArrayConstPtr &error)
{
    std::unique_lock<std::mutex> topicLock(m_mutex);
    m_topicErrors = error->info[0].values;

    for(int i =0; i< m_topicErrors.size();i++)
    {

      /*In local perspective: ie within last clearTimer update,how many times a particaular error occured
        To be precise,to avoid duplication of same error pushed back for publishing*/       
        bool topicStatus = false;
        std::vector<std::string>::iterator it;
        it = std::find (m_topicErrorKey.begin(), m_topicErrorKey.end(), m_topicErrors[i].key); 
        if(it == m_topicErrorKey.end())
        {
            topicStatus= true;
        }
        else
        {
            //ROS_WARN("Topic Duplicated");
        }

        /*In global perspective : how many times a particular error reported*/
        if(m_topicErrorList.find(m_topicErrors[i].key) != m_topicErrorList.end())
        {
            int currentCount = m_topicErrorList[m_topicErrors[i].key];
            m_topicErrorList[m_topicErrors[i].key] =currentCount + 1; 
            //ROS_WARN("%s has occured %d times",m_topicErrors[i].key.c_str(),currentCount);
        }
        else
        {
            ROS_WARN("First time topic occurance  : %s",m_topicErrors[i].key.c_str());
            m_topicErrorList[m_topicErrors[i].key] = 1;
        }

        if((m_topicErrorList[m_topicErrors[i].key] > m_topicMaxErrorOccurences ) && (topicStatus == true))
        {
            m_topicErrorKey.push_back(m_topicErrors[i].key);
            m_errorQueue.push(m_topicErrors[i]);
            ROS_ERROR("Topic error %s has crossed limits",m_topicErrors[i].key.c_str());
        }
    }
    std::cout <<""<<std::endl;
}

void DiagnosticsClient::SystemDiagnosticsCallback(const monitoring_msgs::MonitoringArrayConstPtr &error)
{
    std::unique_lock<std::mutex> systemLock(m_mutex);
    m_systemErrors = error->info[0].values;

    for(int i =0; i< m_systemErrors.size();i++)
    {
        /*In local perspective: ie within last clearTimer update,how many times a particaular error occured
        To be precise,to avoid duplication of same error pushed back for publishing*/       
        bool systemStatus = false;
        std::vector<std::string>::iterator it;
        it = std::find (m_systemErrorKey.begin(), m_systemErrorKey.end(), m_systemErrors[i].key); 
        if(it == m_systemErrorKey.end())
        {
            systemStatus = true;

        }
        else
        {
            //ROS_WARN("System Duplicated");
        }



        if(m_systemErrorList.find(m_systemErrors[i].key) != m_systemErrorList.end())
        {
            int currentCount = m_systemErrorList[m_systemErrors[i].key];
            m_systemErrorList[m_systemErrors[i].key] =currentCount + 1; 
            //ROS_WARN("%s has occured %d times",m_systemErrors[i].key.c_str(),currentCount);
        }
        else
        {
            ROS_WARN("First time system occurance  : %s",m_systemErrors[i].key.c_str());
            m_systemErrorList[m_systemErrors[i].key] = 1;
        }

        if((m_systemErrorList[m_systemErrors[i].key] > m_systemMaxErrorOccurences ) &&(systemStatus == true))
        {
            m_systemErrorKey.push_back(m_systemErrors[i].key);
            m_errorQueue.push(m_systemErrors[i]);
            ROS_ERROR("System error %s has crossed limits",m_systemErrors[i].key.c_str());
        }        
    }
    std::cout <<""<<std::endl;
}

void DiagnosticsClient::SensorDiagnosticsCallback(const monitoring_msgs::MonitoringArrayConstPtr &error)
{
    
    std::unique_lock<std::mutex> sensorLock(m_mutex);
    m_sensorErrors = error->info[0].values;
    for(int i =0; i< m_sensorErrors.size();i++)
    {
        /*In local perspective: ie within last clearTimer update,how many times a particaular error occured
        To be precise,to avoid duplication of same error pushed back for publishing*/           
        bool sensorStatus = false;
        std::vector<std::string>::iterator it;
        it = std::find (m_sensorErrorKey.begin(), m_sensorErrorKey.end(), m_sensorErrors[i].key); 
        if(it == m_sensorErrorKey.end())
        {       
            sensorStatus = true;
        }
        else
        {
            //ROS_WARN("Sensor Duplicated");
        }


        if(m_sensorErrorList.find(m_sensorErrors[i].key) != m_sensorErrorList.end())
        {
            int currentCount = m_sensorErrorList[m_sensorErrors[i].key];
            m_sensorErrorList[m_sensorErrors[i].key] =currentCount + 1; 
            //ROS_WARN("%s has occured %d times",m_topicErrm_sensorErrorsors[i].key.c_str(),currentCount);
        }
        else
        {
            ROS_WARN("First time sensor occurance  : %s",m_sensorErrors[i].key.c_str());
            m_sensorErrorList[m_sensorErrors[i].key] = 1;
        }

        if((m_sensorErrorList[m_sensorErrors[i].key] > m_sensorMaxErrorOccurences ) && (sensorStatus == true))
        {
            m_sensorErrorKey.push_back(m_sensorErrors[i].key);
            m_errorQueue.push(m_sensorErrors[i]);
            ROS_ERROR("Sensor error %s has crossed limits",m_sensorErrors[i].key.c_str());
        }
    }
    std::cout <<""<<std::endl;
}

void DiagnosticsClient::clearQueueTimerCallback(const ros::TimerEvent &e)
{
    m_nodeErrorKey.clear();
    m_topicErrorKey.clear();
    m_systemErrorKey.clear();
    m_sensorErrorKey.clear();
    m_errorQueue = std::priority_queue<monitoring_msgs::KeyValue, std::vector<monitoring_msgs::KeyValue>, CompareError>();

    ROS_ERROR("Clear queue timer");
}


void DiagnosticsClient::displayCriticalErrors()
{
    auto errorQueue = m_errorQueue;
    while (!errorQueue.empty()) 
    { 
        monitoring_msgs::KeyValue p = errorQueue.top(); 
        errorQueue.pop(); 
        std::cout << p.key << "   " << p.errorlevel << "    size "  << m_errorQueue.size()<<std::endl; 
    } 
}
// void DiagnosticsClient::nodeTimerCallback(const ros::TimerEvent &e)
// {
//     for(int i =0; i< m_nodeErrors.size();i++)
//     {
//         if(m_nodeErrorList.find(m_nodeErrors[i].key) != m_nodeErrorList.end())
//         {
//             int currentCount = m_nodeErrorList[m_nodeErrors[i].key];
//             m_nodeErrorList[m_nodeErrors[i].key] =currentCount + 1; 
//             //ROS_WARN("%s has occured %d times",m_nodeErrors[i].key.c_str(),currentCount);
//         }
//         else
//         {
//             //ROS_WARN("First time occurance  : %s",m_nodeErrors[i].key.c_str());
//             m_nodeErrorList[m_nodeErrors[i].key] = 1;
//         }

//         if(m_nodeErrorList[m_nodeErrors[i].key] > m_nodeMaxErrorOccurences )
//         {
//             errorCategorization(m_nodeErrors[i]);

//             // int charPosition = nodeErrorKey.find(":");
//             // std::string extractedKey = nodeErrorKey.substr(charPosition+1);
//             // std::string errorCriticality;
//             // // /std::cout <<"Original  :"<< nodeErrorKey << "Extracted   :"<<  std::endl; 
//             // m_analyzer->getMonitorType(extractedKey,errorCriticality);
//             // ROS_WARN("Criticality of %s is %s",extractedKey.c_str(),errorCriticality.c_str());

//         }
//     }
    
// }






// void DiagnosticsClient::topicTimerCallback(const ros::TimerEvent &e)
// {
//     for(int i =0; i< m_topicErrors.size();i++)
//     {
//         if(m_topicErrorList.find(m_topicErrors[i].key) != m_topicErrorList.end())
//         {
//             int currentCount = m_topicErrorList[m_topicErrors[i].key];
//             m_topicErrorList[m_topicErrors[i].key] =currentCount + 1; 
//             //ROS_WARN("%s has occured %d times",m_topicErrors[i].key.c_str(),currentCount);
//         }
//         else
//         {
//             //ROS_WARN("First time occurance  : %s",m_topicErrors[i].key.c_str());
//             m_topicErrorList[m_topicErrors[i].key] = 1;
//         }

//         if(m_topicErrorList[m_topicErrors[i].key] > m_topicMaxErrorOccurences )
//         {
//             ROS_ERROR("Topic error %s has crossed limits",m_topicErrors[i].key.c_str());
//         }
//     }
    
// }





// void DiagnosticsClient::systemTimerCallback(const ros::TimerEvent &e)
// {
//     for(int i =0; i< m_systemErrors.size();i++)
//     {
//         if(m_systemErrorList.find(m_systemErrors[i].key) != m_systemErrorList.end())
//         {
//             int currentCount = m_systemErrorList[m_systemErrors[i].key];
//             m_systemErrorList[m_systemErrors[i].key] =currentCount + 1; 
//             //ROS_WARN("%s has occured %d times",m_systemErrors[i].key.c_str(),currentCount);
//         }
//         else
//         {
//             //ROS_WARN("First time occurance  : %s",m_systemErrors[i].key.c_str());
//             m_systemErrorList[m_systemErrors[i].key] = 1;
//         }

//         if(m_systemErrorList[m_systemErrors[i].key] > m_systemMaxErrorOccurences )
//         {
//             ROS_ERROR("System error %s has crossed limits",m_systemErrors[i].key.c_str());
//         }
//     }
    
// }






// void DiagnosticsClient::sensorTimerCallback(const ros::TimerEvent &e)
// {
//     for(int i =0; i< m_sensorErrors.size();i++)
//     {
//         if(m_sensorErrorList.find(m_sensorErrors[i].key) != m_sensorErrorList.end())
//         {
//             int currentCount = m_sensorErrorList[m_sensorErrors[i].key];
//             m_sensorErrorList[m_sensorErrors[i].key] =currentCount + 1; 
//             //ROS_WARN("%s has occured %d times",m_topicErrm_sensorErrorsors[i].key.c_str(),currentCount);
//         }
//         else
//         {
//             //ROS_WARN("First time occurance  : %s",m_sensorErrors[i].key.c_str());
//             m_sensorErrorList[m_sensorErrors[i].key] = 1;
//         }

//         if(m_sensorErrorList[m_sensorErrors[i].key] > m_sensorMaxErrorOccurences )
//         {
//             ROS_ERROR("Sensor error %s has crossed limits",m_sensorErrors[i].key.c_str());
//         }
//     }
    
// }




void DiagnosticsClient::errorCategorization(monitoring_msgs::KeyValue &nodeError)
{
    double error_level = nodeError.errorlevel;
    std::string error_key = std::string(nodeError.key);
    if(error_level > 0.8)
    {
        ROS_WARN("Nodes in critical condition : %s",error_key.c_str());
    }
    else if((error_level > 0.6) && (error_level <= 0.8))
    {
        ROS_INFO("Nodes in not so critical condition : %s",error_key.c_str());
    }    
}

/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "diagnostics_client");
/* Create an object to DiagnosticsActionServer */
    
    DiagnosticsClient diagnosticsClient;

    ros::NodeHandle n;
    ros::Rate rate(1);
    
    while(ros::ok())
    {
        //diagnosticsClient.print_queue(diagnosticsClient.m_errorQueue);
        diagnosticsClient.displayCriticalErrors();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}