#ifndef COLUGOBROKER_H
#define COLUGOBROKER_H
#include <utill.h>
#include <sys/types.h>


namespace ColugoBrokerModule
{
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
  
    LinkMessageBase* uplink_ = NULL;
    LinkMessageBase* downlink_ = NULL ;
    

public:

    /**
     * Singletons should not be cloneable.
     */
    ColugoBrokerManager(ColugoBrokerManager &other) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator=(const ColugoBrokerManager &) = delete;

    
    static void Delete(){ delete singleton_ ;} ;

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

    static ColugoBrokerManager *GetInstance(); 
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

      static bool Start() ;
      static bool Stop() ;
    /**
     * @todo forward Mavlink message from Flight controller to companion computer according to filtered and predefined type
     *
     * @param other TODO
     * @return TODO
     */
    bool DownlinkMessage(mavlink_message_t& message) ;
    
    /**
     * @todo forward Mavlink message from companion computer according to filtered and predefined type to Flight controller

     *
     * @param other TODO
     * @return TODO
     */

    void UplinkMessage(mavlink_message_t& message) ;
 };
}

#endif // COLUGOBROKER_H
