#include "analyser.h"



Analyser::Analyser(ros::NodeHandle &nh)
{
    
    nh.getParam("/node_monitor", m_monitorParams);




}


Analyser::~Analyser()
{

}



void Analyser::getMonitorKey(std::string &key)
{

}

void Analyser::getMonitorType(const std::string &key,std::string &type)
{
    if (m_monitorParams.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
 
        for(int i=0; i < m_monitorParams.size(); i++)
        {
            XmlRpc::XmlRpcValue topicObject = m_monitorParams[i];
            
            //std::cout << topicObject["key"] << " : " << topicObject["type"] << " : "<< topicObject["action"] << std::endl;;
            if(topicObject["key"] == key)
            {
                type = std::string(topicObject["type"]);
            }
        }
    }
}

void Analyser::getMonitorAction(std::string &action)
{
    
}
