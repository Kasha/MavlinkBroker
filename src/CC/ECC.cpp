#include <utill.h>
#include <ECC.h>
using namespace ColugoBrokerModule ;

            
bool DownlinkData::Push(mavlink_message_t& message)
{ 
    /* 
    TODO
    ADD Elbit Extended Messages Recived Hndler
    */
   
#ifdef DEBUG
            debug_print("\nUpperlinkData::processMessage::read_messages success\n"); 
			debug_print("\nUpperlinkData::processMessage message.sysid = %i", message.sysid); 
			debug_print("\nUpperlinkData::processMessage message.msgid = %i", message.msgid); 
			debug_print("\nUpperlinkData::processMessage message.compid = %i", message.compid); 
    #endif
    return false ;
}

mavlink_message_t& DownlinkData::Pop()
{
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


bool UpperlinkData::Push(mavlink_message_t& message)
{ 
    /* 
    TODO
    ADD Elbit Extended Messages Recived Hndler
    */
   
#ifdef DEBUG
            debug_print("\nUpperlinkData::processMessage::read_messages success\n"); 
			debug_print("\nUpperlinkData::processMessage message.sysid = %i", message.sysid); 
			debug_print("\nUpperlinkData::processMessage message.msgid = %i", message.msgid); 
			debug_print("\nUpperlinkData::processMessage message.compid = %i", message.compid); 
    #endif
    return false ;
}

mavlink_message_t& UpperlinkData::Pop()
{
   mavlink_message_t message ;
   return message ;
}
