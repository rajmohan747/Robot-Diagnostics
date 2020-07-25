#include "topic_statistics.h"


std::string GenericSubsriber::md5 = "*";

std::string GenericSubsriber::data_type = "/";

/**
* @brief  Constructor for the TopicStatistics
*/


TopicStatistics::TopicStatistics(ros::NodeHandle &nh,std::string topicName,double topicFrequency,std::shared_ptr<Monitor> monitor)
{
    m_topic                 = topicName;
    m_expectedFrequency     = topicFrequency;
    m_monitor               = monitor;
    nh.getParam("/minAcceptableFrequencyFactor", m_minAcceptableFrequencyFactor);
    ROS_WARN("Frequency statsistics constructor initialized with for %s with expected frequency of  %f Hz",m_topic.c_str(),topicFrequency);
    
    /*Subscribers*/
    universalSub = nh.subscribe(m_topic, 1, &TopicStatistics::genericMessageCallback, this);

    double timerUpdateFrequency = std::min(1.0,m_expectedFrequency);    
    /*Timer*/
    topicStatusTimer = nh.createTimer(ros::Duration(1.0 / timerUpdateFrequency), &TopicStatistics::timerCallback, this);
    
    /*Initialization of time variables*/
    m_currentTime = m_lastTime = millis();
    m_startTime   = m_endTime  = millis();

}

/**
* @brief  Destructor for the TopicStatistics
*/

TopicStatistics::~TopicStatistics()
{

}


/**
* @brief  Timer call back for the TopicStatistics
*/
void TopicStatistics::timerCallback(const ros::TimerEvent &e)
{
    if(m_setup)
    {
        //ROS_INFO("Topic %s is updating with %f Hz",m_topic.c_str(),m_averageFrequency);
        if(m_currentSize == m_lastSize)
        {
            ROS_WARN("No data received for the topic : %s",m_topic.c_str());
            //ROS_WARN("No data received current : %d last :%d",m_currentSize,m_lastSize);
            publishNoTopicInfo();
        }
            
        
        if(m_averageFrequency < m_minAcceptableFrequencyFactor*m_expectedFrequency)
        {
            ROS_ERROR("Message updation of %s is slow with : %f",m_topic.c_str(),m_averageFrequency);
            publishTopicDelayInfo();
        }
        m_lastSize = m_currentSize;
        m_endTime  = m_startTime;
    }
}

/**
* @brief  Generic callback function for any topic
*/
void TopicStatistics::genericMessageCallback(const GenericSubsriber &data)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_currentTime = millis();
    m_deltaTime = m_currentTime - m_lastTime;

    m_timeDifferences.push_back(m_deltaTime);
    m_lastTime    = m_currentTime;


    if(m_currentSize > pow(2,20))
    {
        m_timeDifferences.clear();
        m_timeDifferences.resize(0);
        m_currentSize = m_lastSize = 0;
        ROS_ERROR("Resetting the counters");
    }

    m_currentSize = m_timeDifferences.size();
    
    if(m_currentSize > 5)
    {
        std::vector<double> lastNTimeDifferences (m_timeDifferences.end() - 5, m_timeDifferences.end());
        double averageTime  = accumulate(lastNTimeDifferences.begin(), lastNTimeDifferences.end(), 0)/lastNTimeDifferences.size();
        m_averageFrequency  = 1000.0/averageTime;
        m_setup   = true;
     }

}


/**
* @brief  Publishes the error level when topic is not available
*/

void TopicStatistics::publishNoTopicInfo()
{
    std:: string value =" not available";
    m_monitor->addValue(m_topic, value, "", 0.6, AggregationStrategies::FIRST);
}

/**
* @brief  Publishes the error level when topic arrives with a delay
*/

void TopicStatistics::publishTopicDelayInfo()
{
    std:: string value =" delay in  publishing data";
    m_monitor->addValue(m_topic, value, "", 0.3, AggregationStrategies::FIRST);   
}

/**
* @brief  Access the system time using chrono library
* @return time in milliseconds
*/
uint64_t TopicStatistics::millis() 
{
	uint64_t ms =std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
	return ms;
}

