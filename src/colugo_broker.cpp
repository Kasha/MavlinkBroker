#include <utill.h>
#include <colugo_broker.h>
using namespace ColugoBrokerModule ;

/**
 * Static methods should be defined outside the class.*/
ColugoBrokerManager *ColugoBrokerManager::GetInstance()
{
    /**
     * This is a safer way to create an instance. instance = new Singleton is
     * dangeruous in case two instance threads wants to access at the same time
     */
    if(singleton_==nullptr){
        singleton_ = new ColugoBrokerManager();
    }
    return singleton_;
}

void ColugoBrokerManager::read_messages(mavlink_message_t message)
{
	

	

	return;
}
