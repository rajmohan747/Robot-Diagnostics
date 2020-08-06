
#include "diagnostics_aggregator.h"


DiagnosticsAggregator::DiagnosticsAggregator():m_sychronizer(m_synchronizerPolicy(10),nodeSubscriber,topicSubscriber,systemSubscriber,sensorSubscriber)
{
    ROS_INFO("Constructor of DiagnosticsAggregator called");
    /*Parameters from parameter server*/
    nh.getParam("/nodeMaxErrorOccurences", m_nodeMaxErrorOccurences);
    nh.getParam("/topicMaxErrorOccurences", m_topicMaxErrorOccurences);
    nh.getParam("/systemMaxErrorOccurences", m_systemMaxErrorOccurences);
    nh.getParam("/sensorMaxErrorOccurences", m_sensorMaxErrorOccurences);

    /*Subscribers*/
    nodeSubscriber.subscribe(nh, "/node_monitoring/monitoring", 1);
    topicSubscriber.subscribe(nh, "/topic_monitoring/monitoring", 1);
    systemSubscriber.subscribe(nh, "/system_monitoring/monitoring", 1);
    sensorSubscriber.subscribe(nh, "/sensor_monitoring/monitoring", 1);

    /*Publishers*/
    criticalPub      = nh.advertise<monitoring_msgs::KeyValues>("/critical_errors",1);
    nonCriticalPub   = nh.advertise<monitoring_msgs::KeyValues>("/noncritical_errors",1);
    
    /*Timer*/
    double clearErrorFrequency = 0.25;
    clearQueueTimer = nh.createTimer(ros::Duration(1.0/clearErrorFrequency), &DiagnosticsAggregator::clearQueueTimerCallback, this);

    //*Approximate Time Sychronizer*/
    m_sychronizer.registerCallback(boost::bind(&DiagnosticsAggregator::errorCallback,this, _1, _2, _3, _4));
  

}


DiagnosticsAggregator::~DiagnosticsAggregator()
{

}

void DiagnosticsAggregator::errorCallback(const monitoring_msgs::MonitoringArrayConstPtr& nodeError,
    const monitoring_msgs::MonitoringArrayConstPtr& topicError,
    const monitoring_msgs::MonitoringArrayConstPtr& systemError,
    const monitoring_msgs::MonitoringArrayConstPtr& sensorError)
{
  
    std::unique_lock<std::mutex> errorLock(m_mutex);

    m_nodeErrors   = nodeError->info[0].values;
    m_topicErrors  = topicError->info[0].values;
    m_systemErrors = systemError->info[0].values;
    m_sensorErrors = sensorError->info[0].values;
    ROS_WARN("Node :%d  Topic : %d System: %d Sensor %d",m_nodeErrors.size(),
    m_topicErrors.size(),m_systemErrors.size(),m_sensorErrors.size());


    /*Whenever there happens a  change in the message received from node/topic/system/sensor
    the priority queue will be resetted*/
    bool nodeReset   = errorMessageUpdate(m_nodeErrors,m_nodeErrorsLast);
    bool topicReset  = errorMessageUpdate(m_topicErrors,m_topicErrorsLast);
    bool systemReset = errorMessageUpdate(m_systemErrors,m_systemErrorsLast);
    bool sensorReset = errorMessageUpdate(m_sensorErrors,m_sensorErrorsLast); 

    if(nodeReset || topicReset || systemReset || sensorReset)
    {
        ROS_ERROR("New paameter,reset priority queue");
        resetParameters();
    }
    for(int i =0; i< m_nodeErrors.size();i++)
    {
        errorAnalysis(m_nodeErrorKey,m_nodeErrorCountList,m_nodeErrors[i],m_nodeMaxErrorOccurences);
    }

    for(int i =0; i< m_topicErrors.size();i++)
    {
        errorAnalysis(m_topicErrorKey,m_topicErrorCountList,m_topicErrors[i],m_topicMaxErrorOccurences);
    }

    for(int i =0; i< m_systemErrors.size();i++)
    {
        errorAnalysis(m_systemErrorKey,m_systemErrorCountList,m_systemErrors[i],m_systemMaxErrorOccurences);
    }

    for(int i =0; i< m_sensorErrors.size();i++)
    {
        errorAnalysis(m_sensorErrorKey,m_sensorErrorCountList,m_sensorErrors[i],m_sensorMaxErrorOccurences);
    }
    std::cout <<""<<std::endl;    

    m_nodeErrorsLast   = m_nodeErrors;
    m_topicErrorsLast  = m_topicErrors;
    m_systemErrorsLast = m_systemErrors;
    m_sensorErrorsLast = m_sensorErrors;

}


bool DiagnosticsAggregator::errorMessageUpdate(const std::vector<monitoring_msgs::KeyValue> &errors,const std::vector<monitoring_msgs::KeyValue> &errorsLast)
{
    
    if(errors.size() != errorsLast.size())
    {
        return true;
    }
    for(int i =0; i< errors.size();i++)
    {
        if((errors[i].key != errorsLast[i].key) || (errors[i].errorlevel != errorsLast[i].errorlevel))
        {
            return true;
        }
    }
    return false;
}


/**
 * @brief If the error from the nodes/topics/system/sensor happens more than 
 *maxErrorOccurence times,it will be updated in the priority queue.
*/

void DiagnosticsAggregator::errorAnalysis(std::vector<std::string> &errorKey,std::unordered_map<std::string,int> &errorCountList,monitoring_msgs::KeyValue &errorKeyValue,int maxErrorOccurence)
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


/**
 * @brief Timer callback for clearing/updating the errors from the  nodes/topics/systems/sensors
*/

void DiagnosticsAggregator::clearQueueTimerCallback(const ros::TimerEvent &e)
{
    ROS_ERROR("Clear queue timer : %d", m_errorQueue.size());
    resetParameters();
    
}

/**
 * @brief resets the priority queue and the callback key lists
*/
void DiagnosticsAggregator::resetParameters()
{
   /*clearing all the keys list in 'clearErrorFrequency' Hz*/
    m_nodeErrorKey.clear();
    m_topicErrorKey.clear();
    m_systemErrorKey.clear();
    m_sensorErrorKey.clear();

    /*clearing the priority queue*/
    m_errorQueue = std::priority_queue<monitoring_msgs::KeyValue, std::vector<monitoring_msgs::KeyValue>, CompareError>();
}

/**
 * @brief Categorizes the priorty queue-errors based on criticality and publishes the message.
*/
void DiagnosticsAggregator::errorCategorization()
{
    std::unique_lock<std::mutex> categoryLock(m_mutex);

    monitoring_msgs::KeyValues nonErrors;

    m_criticalErrors.keyvalues.clear();
    m_nonCriticalErrors.keyvalues.clear();

    m_criticalErrors.keyvalues.resize(0);
    m_nonCriticalErrors.keyvalues.resize(0);


    auto errorQueue = m_errorQueue;
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
        }
        
    
    } 

    /*Ensuring the publishing happens whenever valid errors are available*/
    if(m_errorQueue.size() > 0)
    {
        criticalPub.publish(m_criticalErrors);
        nonCriticalPub.publish(m_nonCriticalErrors);
        
        ROS_INFO("Critical errors : %d",m_criticalErrors.keyvalues.size());
        ROS_INFO("Non Critical errors : %d",m_nonCriticalErrors.keyvalues.size());
        ROS_INFO("Non errors : %d",nonErrors.keyvalues.size());
    }

    

}



/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "diagnostics_aggregrator");
    ROS_INFO("Main of DiagnosticsAggregator called");
    /* Create an object to DiagnosticsAggregator */
    DiagnosticsAggregator diagnosticsAggregator;

    ros::NodeHandle n;
    ros::Rate rate(1);
    
    while(ros::ok())
    {
        diagnosticsAggregator.errorCategorization();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
