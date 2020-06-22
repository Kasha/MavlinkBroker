#include <utill.h>
#include <colugo_companion_computer.h>
#include <colugo_broker.h>

using namespace ColugoBrokerModule ;

ColugoBrokerManager::ColugoBrokerManager(): uplink_(new UpperlinkData()), downlink_(new DownlinkData())
{
    
} ;

ColugoBrokerManager::~ColugoBrokerManager()
{
    delete uplink_ ;
    delete downlink_ ;
} ;

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

/**
 * Static methods should be defined outside the class.*/
bool ColugoBrokerManager::Start()
{
    ColugoBrokerManager* oColugoBrokerManager = GetInstance();
    return true ;
}

bool ColugoBrokerManager::Stop()
{
    Delete() ;
    return true ;
}


bool ColugoBrokerManager::DownlinkMessage(mavlink_message_t& message)
{
    if( message.sysid == 0 )
    {
        return false ;
    }
    #ifdef DEBUG
    debug_print("\n\nColugoBrokerManager::DownlinkMessage success\n"); 
    debug_print("\nColugoBrokerManager::DownlinkMessage = %i", message.sysid); 
    debug_print("\nColugoBrokerManager::DownlinkMessage = %i", message.msgid); 
    debug_print("\nColugoBrokerManager::DownlinkMessage = %i\n\n", message.compid); 
    #endif
    return true ;
}

void ColugoBrokerManager::UplinkMessage(mavlink_message_t& message)
{
}
