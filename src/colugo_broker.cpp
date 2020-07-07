#include <utill.h>
//#include <colugo_companion_computer.h>
//#include <colugo_broker.h>

using namespace ColugoBrokerModule ;

//COLUGO_BROKER_INIT_ENUM ColugoBrokerManager::stopStreamMessages_= COLUGO_BROKER_INIT_NONE ;

ColugoBrokerManager::ColugoBrokerManager(): uplink_(make_shared<UpperlinkData>()/**/), downlink_(make_shared<DownlinkData>())
{
    stopStreamMessages_ = COLUGO_BROKER_INIT_NONE ;
    singleton_ = nullptr ;
} ;

ColugoBrokerManager::~ColugoBrokerManager()
{
    stopStreamMessages_ = COLUGO_BROKER_INIT_NONE ;
}

/**
Static methods should be defined outside the class.
**/

ColugoBrokerManager *ColugoBrokerManager::GetInstance(bool bAllowInit)
{
    /**
     * This is a safer way to create an instance. instance = new Singleton is
     * dangeruous in case two instance threads wants to access at the same time
     */
    
    /*Allow Singleton creation only from Start, to control allocation and deletion and allow proper delete when downtime and Messages continue to arrive and should be stopped or kept in DB*/
    if ( singleton_== nullptr && bAllowInit == true ) 
    {
        singleton_ = new ColugoBrokerManager();
        stopStreamMessages_ = COLUGO_BROKER_START ;
    }
    
    if( singleton_ == nullptr )
    {
        stopStreamMessages_ = COLUGO_BROKER_STOP ;
        throw(COLUGO_BROKER_STOP) ;
    }
    
    return singleton_;
}

/**
 * Static methods should be defined outside the class.*/
COLUGO_BROKER_INIT_ENUM ColugoBrokerManager::Start()
{
    /*Allow Singleton creation only from Start, to control allocation and deletion and allow proper delete when downtime and Messages continue to arrive and should be stopped or kept in DB*/
    ColugoBrokerManager* oColugoBrokerManager = GetInstance(true);
    return stopStreamMessages_ ;
}

COLUGO_BROKER_INIT_ENUM ColugoBrokerManager::Stop()
{
    Delete() ;
    stopStreamMessages_ = COLUGO_BROKER_STOP ;
    return stopStreamMessages_ ;
}

COLUGO_BROKER_INIT_ENUM ColugoBrokerManager::Status()
{
    return stopStreamMessages_ ;
}

/*Receiving Mavlink from Flight controller and forward data to client according to implemented DownlinkData*/
bool ColugoBrokerManager::DownlinkMessage( mavlink_message_t& message )
{
    if ( message.sysid == 0 ) 
    {
        return false ;
    }
    
    if( Status() == COLUGO_BROKER_START )
    {
        ColugoBrokerManager* oColugoBrokerManager = GetInstance();
        oColugoBrokerManager->downlink_->Push(message) ;
#ifdef DEBUG
        debug_print ( "\n\nColugoBrokerManager::DownlinkMessage success\n" );
        debug_print ( "\nColugoBrokerManager::DownlinkMessage = %i", message.sysid );
        debug_print ( "\nColugoBrokerManager::DownlinkMessage = %i", message.msgid );
        debug_print ( "\nColugoBrokerManager::DownlinkMessage = %i\n\n", message.compid );
#endif
    }
    else
    {
        /*Process is on Downtime or didn't finish loading (Start Process)
         * Keep data in DB? write log
        */
        return false ;
    }
    
    return true ;
}

/*Receiving Buffer from Client and forward to UpperlinkData to convert to Mavlink Message and send it to Flight Controller*/
bool ColugoBrokerManager::UplinkMessage( const char* buffer_message )
{
    /* if ( message.sysid == 0 ) 
    {
        return false ;
    }*/
    
    if( Status() == COLUGO_BROKER_START )
    {
        ColugoBrokerManager* oColugoBrokerManager = GetInstance();
        oColugoBrokerManager->uplink_->Push(buffer_message) ;
        
#ifdef DEBUG
        /*debug_print ( "\n\nColugoBrokerManager::DownlinkMessage success\n" );
        debug_print ( "\nColugoBrokerManager::DownlinkMessage = %i", message.sysid );
        debug_print ( "\nColugoBrokerManager::DownlinkMessage = %i", message.msgid );
        debug_print ( "\nColugoBrokerManager::DownlinkMessage = %i\n\n", message.compid );*/
#endif
    }
    else
    {
        /*Process is on Downtime or didn't finish loading (Start Process)
         * Keep data in DB? write log
        */
        return false ;
    }
    
    return true ;
}

