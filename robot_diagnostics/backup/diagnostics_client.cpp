
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

    criticalPub      = nh.advertise<monitoring_msgs::KeyValues>("/critical_errors",1);
    nonCriticalPub   = nh.advertise<monitoring_msgs::KeyValues>("/noncritical_errors",1);
    double clearErrorFrequency = 0.25;
    clearQueueTimer = nh.createTimer(ros::Duration(1.0/clearErrorFrequency), &DiagnosticsClient::clearQueueTimerCallback, this);

    //m_analyzer =std::make_shared<Analyser>(nh);

}


DiagnosticsClient::~DiagnosticsClient()
{

}


/**
 * @brief If the error from the nodes/topics/system/sensor happens more than 
 *maxErrorOccurence times,it will be updated in the priority queue.
*/

void DiagnosticsClient::errorAnalysis(std::vector<std::string> &errorKey,std::unordered_map<std::string,int> &errorCountList,monitoring_msgs::KeyValue &errorKeyValue,int maxErrorOccurence)
{
    /*In local perspective: ie, will be used for aggregating node/topic/system/sensor crital errors.
    Here we are making sure that multiple copies of same data is not created and the
    list is cleared in every clearErrorFrequency interval*/
    bool status = false;
    std::vector<std::string>::iterator it;
    it = std::find (errorKey.begin(), errorKey.end(), errorKeyValue.key); 
    if(it == errorKey.end())
    {
        status = true;
    }



        /*In global perspective : how many times a particular error reported*/
    if(errorCountList.find(errorKeyValue.key) != errorCountList.end())
    {
        int currentCount = errorCountList[errorKeyValue.key];
        errorCountList[errorKeyValue.key] =currentCount + 1; 
        //ROS_WARN("%s has occured %d times",m_nodeErrors[i].key.c_str(),currentCount);
    }
    else
    {
        //ROS_WARN("First time occurance  : %s",errorKeyValue.key.c_str());
        errorCountList[errorKeyValue.key] = 1;
    }

    if((errorCountList[errorKeyValue.key] > maxErrorOccurence ) && (status == true))
    {
        
        errorKey.push_back(errorKeyValue.key);
        m_errorQueue.push(errorKeyValue);
       // ROS_ERROR(" Error %s has crossed limits",errorKeyValue.key.c_str());
    }
}




bool DiagnosticsClient::compareErrors(const std::vector<monitoring_msgs::KeyValue> &errors,const std::vector<monitoring_msgs::KeyValue> &errorsLast)
{
    if(errors.size() != errorsLast.size())
    {
        return false;
    }
    else
    {
        for(int i=0;i<errors.size();i++)
        {
            if((errors[i].key != errorsLast[i].key) ||  (errors[i].errorlevel != errorsLast[i].errorlevel) )
            {
                return false;
            }
        }
    }
    return true;
}


/**
 * @brief Subscriber callback for node based errors 
*/

void DiagnosticsClient::NodeDiagnosticsCallback(const monitoring_msgs::MonitoringArrayConstPtr &error)
{

    std::unique_lock<std::mutex> nodeLock(m_mutex);
    m_nodeErrors = error->info[0].values;
    
    ROS_WARN("Node error size : %d",m_nodeErrors.size());

    /*TO DO: Clear /update the m_errorQueue everytime whenever a new callback is called*/
    // bool nodeEquality= true;
    // nodeEquality = compareErrors(m_nodeErrors,m_nodeErrorsLast);
    // if(nodeEquality ==false)
    // {
    //     m_errorQueue = std::priority_queue<monitoring_msgs::KeyValue, std::vector<monitoring_msgs::KeyValue>, CompareError>();
    //     ROS_WARN("Node monitoring callback changed.Resetting priority queue");
    // }

    for(int i =0; i< m_nodeErrors.size();i++)
    {
        errorAnalysis(m_nodeErrorKey,m_nodeErrorCountList,m_nodeErrors[i],m_nodeMaxErrorOccurences);
    }
    std::cout <<""<<std::endl;

    //m_nodeErrorsLast = m_nodeErrors;
    
}

/**
 * @brief Subscriber callback for topic based errors 
*/

void DiagnosticsClient::TopicDiagnosticsCallback(const monitoring_msgs::MonitoringArrayConstPtr &error)
{
    std::unique_lock<std::mutex> topicLock(m_mutex);
    m_topicErrors = error->info[0].values;

    ROS_WARN("Topic error size : %d",m_topicErrors.size());

    /*TO DO: Clear /update the m_errorQueue everytime whenever a new callback is called*/
    // bool topicEquality= true;
    // topicEquality = compareErrors(m_topicErrors,m_topicErrorsLast);
    // if(topicEquality ==false)
    // {
    //     m_errorQueue = std::priority_queue<monitoring_msgs::KeyValue, std::vector<monitoring_msgs::KeyValue>, CompareError>();
    //     ROS_WARN("Topic monitoring callback changed.Resetting priority queue");
    // }

    for(int i =0; i< m_topicErrors.size();i++)
    {
        errorAnalysis(m_topicErrorKey,m_topicErrorCountList,m_topicErrors[i],m_topicMaxErrorOccurences);
    }
    std::cout <<""<<std::endl;
    //m_topicErrorsLast = m_topicErrors;
}

/**
 * @brief Subscriber callback for system based errors 
*/

void DiagnosticsClient::SystemDiagnosticsCallback(const monitoring_msgs::MonitoringArrayConstPtr &error)
{
    std::unique_lock<std::mutex> systemLock(m_mutex);
    m_systemErrors = error->info[0].values;

    /*TO DO: Clear /update the m_errorQueue everytime whenever a new callback is called*/
    // bool systemEquality= true;
    // systemEquality = compareErrors(m_systemErrors,m_systemErrorsLast);
    // if(systemEquality ==false)
    // {
    //     m_errorQueue = std::priority_queue<monitoring_msgs::KeyValue, std::vector<monitoring_msgs::KeyValue>, CompareError>();
    //     ROS_WARN("System monitoring callback changed.Resetting priority queue");
    // }

    for(int i =0; i< m_systemErrors.size();i++)
    {
        errorAnalysis(m_systemErrorKey,m_systemErrorCountList,m_systemErrors[i],m_systemMaxErrorOccurences);
    }
    std::cout <<""<<std::endl;
    //m_systemErrorsLast = m_systemErrors;
}

/**
 * @brief Subscriber callback for sensor based errors 
*/
void DiagnosticsClient::SensorDiagnosticsCallback(const monitoring_msgs::MonitoringArrayConstPtr &error)
{
    
    std::unique_lock<std::mutex> sensorLock(m_mutex);
    m_sensorErrors = error->info[0].values;

    /*TO DO: Clear /update the m_errorQueue everytime whenever a new callback is called*/
    // bool sensorEquality= true;
    // sensorEquality = compareErrors(m_sensorErrors,m_sensorErrorsLast);
    // if(sensorEquality ==false)
    // {
    //     m_errorQueue = std::priority_queue<monitoring_msgs::KeyValue, std::vector<monitoring_msgs::KeyValue>, CompareError>();
    //     ROS_WARN("Sensor monitoring callback changed.Resetting priority queue");
    // }


    for(int i =0; i< m_sensorErrors.size();i++)
    {
        errorAnalysis(m_sensorErrorKey,m_sensorErrorCountList,m_sensorErrors[i],m_sensorMaxErrorOccurences);
    }
    std::cout <<""<<std::endl;

    //m_sensorErrorsLast = m_sensorErrors;
}

/**
 * @brief Timer callback for clearing/updating the errors from the  nodes/topics/systems/sensors
*/

void DiagnosticsClient::clearQueueTimerCallback(const ros::TimerEvent &e)
{
    std::unique_lock<std::mutex> clearLock(m_mutex);
    /*clearing all the keys list in 'clearErrorFrequency' Hz*/
    m_nodeErrorKey.clear();
    m_topicErrorKey.clear();
    m_systemErrorKey.clear();
    m_sensorErrorKey.clear();

    /*clearing the priority queue*/
    m_errorQueue = std::priority_queue<monitoring_msgs::KeyValue, std::vector<monitoring_msgs::KeyValue>, CompareError>();

    ROS_ERROR("Clear queue timer");
}



/**
 * @brief Categorizes the priorty queue-errors based on criticality and publishes the message.
*/
void DiagnosticsClient::errorCategorization()
{
    monitoring_msgs::KeyValues nonErrors;

    m_criticalErrors.keyvalues.clear();
    m_nonCriticalErrors.keyvalues.clear();

    m_criticalErrors.keyvalues.resize(0);
    m_nonCriticalErrors.keyvalues.resize(0);


    auto errorQueue = m_errorQueue;
    ROS_INFO("Error queue size :  %d",m_errorQueue.size());
    while (!errorQueue.empty()) 
    { 
        monitoring_msgs::KeyValue p = errorQueue.top(); 
        errorQueue.pop(); 
        if(p.errorlevel >= 0.8)
        {
            m_criticalErrors.keyvalues.push_back(p);
        }
        else if ((p.errorlevel < 0.8) && (p.errorlevel >= 0.6))
        {
            m_nonCriticalErrors.keyvalues.push_back(p);
        }
        else
        {
            nonErrors.keyvalues.push_back(p);
           // std::cout << p.key << "   " << p.errorlevel << "    size "  << errorQueue.size()<<std::endl;  
        }
        
    
    } 

    /*Ensuring the publishing happens whenever valid errors are available*/
    if(m_errorQueue.size() > 0)
    {
        criticalPub.publish(m_criticalErrors);
        nonCriticalPub.publish(m_nonCriticalErrors);
    }

    
    ROS_INFO("Critical errors : %d",m_criticalErrors.keyvalues.size());
    ROS_INFO("Non Critical errors : %d",m_nonCriticalErrors.keyvalues.size());
    ROS_INFO("Non errors : %d",nonErrors.keyvalues.size());
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
        diagnosticsClient.errorCategorization();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}