#ifndef COLUGOBROKER_H
#define COLUGOBROKER_H
#include <utill.h>
#include <ardupilotmega/mavlink.h>
#include <sys/types.h>
#include <unistd.h>

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
    ColugoBrokerManager()
    {
    }
    

    static ColugoBrokerManager* singleton_;
  
    queue <mavlink_message_t> uplink_ ;
    queue <mavlink_message_t> downlink_ ;
public:

    /**
     * Singletons should not be cloneable.
     */
    ColugoBrokerManager(ColugoBrokerManager &other) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator=(const ColugoBrokerManager &) = delete;
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
    static ColugoBrokerManager *GetInstance(const string& value);
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
    void SomeBusinessLogic()
    {
          #ifdef DEBUG
            debug_print("\nSomeBusinessLogic val=%d", getpid());
          #endif
    }
    void read_messages(mavlink_message_t message) ;
 /**
     * @todo write docs
     *
     * @param other TODO
     * @return TODO
     */
    /*string value() const{
        return value_;
    } */
};
}

#endif // COLUGOBROKER_H
