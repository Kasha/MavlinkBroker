#ifndef ELBIT_COMPANION_COMPUTER
#define ELBIT_COMPANION_COMPUTER

#define ELBIT_MAVLINK_SIZE = 263

namespace ColugoBrokerClass
{
template<typename T>
class Upperlink 
{ 
private: 
     queue <vector<T>> m_oUplink; //Listens to commands for Flight controller
     //queue <shared_ptr<char>> downlink; //Sends commands from Flight controller to GCS or Companion Computer
     
     /*inline shared_ptr<T> MakeArray(int size)
    {
        return shared_ptr<T>( new T[size], []( T *p ){ delete [] p; } );
    }*/
public: 
     Upperlink(){ }
    
     //Upperlink(T &val) delete ;
    
    // Upperlink(queue <shared_ptr<T>> &val) delete ;
     
    //~Upperlink(){m_oUplink.reset() ;} ;
  
    void Push(vector<T> &val)
    {
        m_oUplink.push(val);
    }
   
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
