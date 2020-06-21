#ifndef ELBIT_COMPANION_COMPUTER
#define ELBIT_COMPANION_COMPUTER

#define ELBIT_MAVLINK_SIZE = 263

#include <ECC_UPLINK_MESSAGE.h>

#include <utill.h>

using namespace std ;

namespace ColugoBrokerModule
{
class UpperlinkData 
{ 
protected: 
     queue <string> m_olink; //Listens to commands for Flight controller
     //queue <shared_ptr<char>> downlink; //Sends commands from Flight controller to GCS or Companion Computer
     
     /*inline shared_ptr<T> MakeArray(int size)
    {
        return shared_ptr<T>( new T[size], []( T *p ){ delete [] p; } );
    }*/
public: 
     UpperlinkData(){ }
    
     //Upperlink(T &val) delete ;
    
    // Upperlink(queue <shared_ptr<T>> &val) delete ;
     
    //~Upperlink(){m_oUplink.reset() ;} ;
  
    void push(string val)
    {
        m_olink.push(val);
    }
    
    string &front()
    {
        return m_olink.front();
    }
}; 


class DownlinkData: public UpperlinkData
{ 
private: 
       /*inline shared_ptr<T> MakeArray(int size)
    {
        return shared_ptr<T>( new T[size], []( T *p ){ delete [] p; } );
    }*/
public: 
     DownlinkData(){ }
    
     //Upperlink(T &val) delete ;
    
    // Upperlink(queue <shared_ptr<T>> &val) delete ;
     
    //~Upperlink(){m_oUplink.reset() ;} ;
  

   
   bool processMessage(mavlink_message_t& message) ;
   bool pushMessageBufferToQueue(mavlink_message_t& message) ;
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
