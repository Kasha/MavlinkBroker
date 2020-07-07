#ifndef ELBIT_COMPANION_COMPUTER
#define ELBIT_COMPANION_COMPUTER

#define ELBIT_MAVLINK_SIZE = 263

#include <ECC_UPLINK_MESSAGE.h>

#include <utill.h>

using namespace std ;

namespace ColugoBrokerModule
{

class ElbitlinkBase:public LinkMessageBase
{
public:
    ElbitlinkBase() {} ;
    virtual ~ElbitlinkBase() {}  ;
    virtual bool Push( const char* buffer_message ) ;
    
protected:
    queue <string> m_olink; //Listens to commands for Flight controller
    
    pthread_mutex_t queue_mutex;
} ;

class UpperlinkData:public ElbitlinkBase
{
public:
    UpperlinkData() {} ;
    virtual ~UpperlinkData() {};

    /*Receiving a Mavlink Buffer, convert to Mavlink and forward to Client*/
    virtual bool Push( const char* buffer_message ) ;
    virtual mavlink_message_t Pop() ;
    //Upperlink(T &val) delete ;

    // Upperlink(queue <shared_ptr<T>> &val) delete ;

    //~Upperlink(){m_oUplink.reset() ;} ;

    /*void push(string val)
    {
        m_olink.push(val);
    }*/

    /* string &front()
        {
            return m_olink.front();
        }*/
};


class DownlinkData: public ElbitlinkBase
{
public:
    DownlinkData() {} ;
    virtual ~DownlinkData() {};
    /*Receiving a Mavlink from Flight controller and forward data to client according to implemented DownlinkData*/
    virtual bool Push( const mavlink_message_t& message ) ;
    virtual mavlink_message_t Pop() ;


    /*  // This operator overloading enables calling
        // operator function () on objects of increment
        shared_ptr<T> operator () (char *vUplink) const
        {
            auto  buffer = MakeArray<char>(64);

            auto buffer = std::make_shared<std::array<char, ELBIT_MAVLINK_SIZE+1>>();
            m_oUplink = std::make_shared<char> (ELBIT_MAVLINK_SIZE+1);
            //Excpect for Mavlink size up to ELBIT_MAVLINK_SIZE
            return shared_ptr<char>((char *)vUplink);
        } */
};


}

#endif

