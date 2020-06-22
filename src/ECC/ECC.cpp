#include <utill.h>
#include <ECC.h>
using namespace ColugoBrokerModule ;

bool DownlinkData::processMessage(mavlink_message_t& message)
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

bool DownlinkData::pushMessageBufferToQueue(mavlink_message_t& message)
{
   /*- // Lock
	pthread_mutex_lock(&lock);

	// Write packet via serial link
	const int bytesWritten = static_cast<int>(write(fd, buf, len));

	// Wait until all data has been written
	tcdrain(fd);

	// Unlock
	pthread_mutex_unlock(&lock);


	return bytesWritten;*/
   return true ;
}
