#ifndef COLUGOBROKER_H
#define COLUGOBROKER_H
#include <utill.h>
#include <sys/types.h>


namespace ColugoBrokerModule
{
    enum COLUGO_BROKER_INIT_ENUM
    {
        COLUGO_BROKER_INIT_NONE = 1000 + COLUGO_BROKER_ERROR_GENERAL,
        COLUGO_BROKER_START = 1001 + COLUGO_BROKER_ERROR_GENERAL,
        COLUGO_BROKER_STOP = 1002 + COLUGO_BROKER_ERROR_GENERAL
    } ;
/**
* The Singleton class defines the `GetInstance` method that serves as an
* alternative to constructor and lets clients access the same instance of this
* class over and over.
*/
class ColugoBrokerManager
{

    /**
     * The Singleton's constructor should always be private to prevent direct
     * construction calls with the `new` operator.
     */

protected:

    ColugoBrokerManager() ;
    ~ColugoBrokerManager() ;
    static ColugoBrokerManager* singleton_ ;

    /*Using Shared_Pointer for handling Queue resource lock and deleting resources when out while Mavlink data keeps arriving*/
    shared_ptr<UpperlinkData> uplink_ ;
    shared_ptr<DownlinkData> downlink_  ;
   
    ;/*When Stopping Singleton (deleting it) make sure it won't be created again and uplink_/downlink_ stop receiving data*/
    inline static COLUGO_BROKER_INIT_ENUM stopStreamMessages_ = COLUGO_BROKER_INIT_NONE;
    
public:

    /**
     * Singletons should not be cloneable.
     */
    ColugoBrokerManager ( ColugoBrokerManager &other ) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator = ( const ColugoBrokerManager & ) = delete;


    static void Delete()
    {
        delete singleton_ ;
        singleton_ = nullptr ;
    } ;

    /**
     * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     */
    /**
        * @todo write docs
        *
        * @param other TODO
        * @return TODO
        */
    /*Allow Singleton creation only from Start, to control allocation and deletion and allow proper delete when downtime and Messages continue to arrive and should be stopped or kept in DB*/
    static ColugoBrokerManager *GetInstance(bool bAllowInit = false);
    /**
     * Finally, any singleton should define some business logic, which can be
     * executed on its instance.
     */
    /**
    * @todo write docs
    *
    * @param other TODO
    * @return TODO
    */

    static COLUGO_BROKER_INIT_ENUM Start() ;
    static COLUGO_BROKER_INIT_ENUM Stop() ;
    static COLUGO_BROKER_INIT_ENUM Status() ;
    /**
     * @todo forward Mavlink message from Flight controller to companion computer according to filtered and predefined type
     * @description Receiving Mavlink from Flight controller and forward data to client according to implemented DownlinkData
     * @param other TODO
     * @return TODO
     */
    static bool DownlinkMessage( mavlink_message_t& message ) ;

    /**
     * @todo forward Mavlink message from companion computer according to filtered and predefined type to Flight controller
     * @description Receiving Buffer from Client and forward to UpperlinkData to convert to Mavlink Message and send it to Flight Controller
     *
     * @param other TODO
     * @return TODO
     */

    static bool UplinkMessage( const char* buffer_message ) ;
};

}

#endif // COLUGOBROKER_H

