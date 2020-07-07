#include <utill.h>
#include <ECC.h>
using namespace ColugoBrokerModule ;
  
bool ElbitlinkBase::Push( const char* buffer_message )
{
    pthread_mutex_lock(&queue_mutex);
    
    string sMesg(buffer_message) ;
    m_olink.push(sMesg) ;
    pthread_mutex_unlock(&queue_mutex);
    return false ;
}

bool DownlinkData::Push( const mavlink_message_t& message )
{
    char buffer_message[300];

    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer ( ( uint8_t* ) buffer_message, &message );
    bool bRes = ElbitlinkBase::Push(buffer_message) ;
    return bRes ;
}

mavlink_message_t DownlinkData::Pop()
{
    //return m_olink->pop() ;
    /*- // Lock
    pthread_mutex_lock(&lock);

    // Write packet via serial link
    const int bytesWritten = static_cast<int>(write(fd, buf, len));

    // Wait until all data has been written
    tcdrain(fd);

    // Unlock
    pthread_mutex_unlock(&lock);

    mavlink_message_t& message
    return bytesWritten;*/

    mavlink_message_t message ;
    return message ;
}


bool UpperlinkData::Push( const char* buffer_message )
{
    bool bRes = ElbitlinkBase::Push(buffer_message) ;
    return bRes ;
}

 mavlink_message_t UpperlinkData::Pop()
{
    mavlink_message_t message ;
    return message ;
}

