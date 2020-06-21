/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
          
/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */
      

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}
  

// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;

	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

	printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class 
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Generic_Port *port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 254; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	port = port_; // port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	current_setpoint = setpoint;
}

// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void Autopilot_Interface::read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success ) 
		{    
            ColugoBrokerManager *oColugoBrokerManager = ColugoBrokerManager::GetInstance();
            oColugoBrokerManager->SomeBusinessLogic() ;
          
            //ColugoBrokerManager* oColugoBrokerManager = ColugoBrokerManager::GetInstance("EEC");

            //colugoBrokerManagerST->read_messages(message);
    #ifdef DEBUG
            debug_print("\nAutopilot_Interface::read_messages success\n"); 
			debug_print("\nAutopilot_Interface message.sysid = %i", message.sysid); 
			debug_print("\nAutopilot_Interface message.msgid = %i", message.msgid); 
			debug_print("\nAutopilot_Interface message.compid = %i", message.compid); 
    #endif
		 	
            // Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;


            //uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

            // check message is write length
            //unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
 
            //if( downlink_data.processMessage(message) == false )//If false parse it here
            {
                // Handle Message ID
                switch (message.msgid)
                {

                    case MAVLINK_MSG_ID_HEARTBEAT:
                    {
                        #ifdef DEBUG
                            debug_print("\nMAVLINK_MSG_ID_HEARTBEAT=%d", MAVLINK_MSG_ID_HEARTBEAT);
                        #endif
                    
                        mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
                        current_messages.time_stamps.heartbeat = get_time_usec();
                        this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
                        break;
                    } 

                    case MAVLINK_MSG_ID_SYS_STATUS:
                    {
                        #ifdef DEBUG
                            debug_print("\nMAVLINK_MSG_ID_SYS_STATUS=%d", MAVLINK_MSG_ID_SYS_STATUS);
                        #endif
                        mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
                        current_messages.time_stamps.sys_status = get_time_usec();
                        this_timestamps.sys_status = current_messages.time_stamps.sys_status;
                        break;
                    }

                    case MAVLINK_MSG_ID_BATTERY_STATUS:
                    {
                        mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
                        
                        #ifdef DEBUG
                            debug_print("\nMAVLINK_MSG_ID_BATTERY_STATUS current_messages.battery_status.battery_remaining=%d",  current_messages.battery_status.battery_remaining);
                        #endif
                            
                        current_messages.time_stamps.battery_status = get_time_usec();
                        this_timestamps.battery_status = current_messages.time_stamps.battery_status;
                        break;
                    }

                    case MAVLINK_MSG_ID_RADIO_STATUS:
                    {
                        #ifdef DEBUG
                            debug_print("\nMAVLINK_MSG_ID_RADIO_STATUS=%d", MAVLINK_MSG_ID_RADIO_STATUS);
                        #endif
                            
                        mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
                        current_messages.time_stamps.radio_status = get_time_usec();
                        this_timestamps.radio_status = current_messages.time_stamps.radio_status;
                        break;
                    }

                    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                    {
                        mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
                        
                        #ifdef DEBUG
                            debug_print("\nMAVLINK_MSG_ID_LOCAL_POSITION_NED current_messages.local_position_ned.x=%lf", current_messages.local_position_ned.x);
                        #endif
                        
                        current_messages.time_stamps.local_position_ned = get_time_usec();
                        this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
                        break;
                    }

                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                    {
                        mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
                        
                        #ifdef DEBUG
                        debug_print("\nMAVLINK_MSG_ID_GLOBAL_POSITION_INT current_messages.global_position_int.vx=%d", current_messages.global_position_int.vx);
                        #endif
                            
                        current_messages.time_stamps.global_position_int = get_time_usec();
                        this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
                        break;
                    }

                    case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
                    {
                        mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
                    
                        #ifdef DEBUG
                            debug_print("\nMAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED current_messages.position_target_local_ned.x=%lf", current_messages.position_target_local_ned.x);
                        #endif
                        
                        current_messages.time_stamps.position_target_local_ned = get_time_usec();
                        this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
                        break;
                    }

                    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
                    {
                        mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
                        
                        #ifdef DEBUG
                            debug_print("\nMAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT current_messages.position_target_global_int.vx=%lf", current_messages.position_target_global_int.vx);
                        #endif
                            
                        current_messages.time_stamps.position_target_global_int = get_time_usec();
                        this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
                        break;
                    }

                    case MAVLINK_MSG_ID_HIGHRES_IMU:
                    {
                        #ifdef DEBUG
                            debug_print("\nMAVLINK_MSG_ID_HIGHRES_IMU=%d", MAVLINK_MSG_ID_HIGHRES_IMU);
                        #endif
                        mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
                        current_messages.time_stamps.highres_imu = get_time_usec();
                        this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
                        break;
                    }

                    case MAVLINK_MSG_ID_ATTITUDE:
                    {
                        #ifdef DEBUG
                            debug_print("\nMAVLINK_MSG_ID_ATTITUDE=%d", MAVLINK_MSG_ID_ATTITUDE);
                        #endif
                        mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
                        current_messages.time_stamps.attitude = get_time_usec();
                        this_timestamps.attitude = current_messages.time_stamps.attitude;
                        break;
                    }

                    default:
                    {
                        #ifdef DEBUG
                            debug_print("\nWarning, did not handle message id = %i",message.msgid);
                        #endif
                            
                        break;
                    }


                } // end: switch msgid
            }

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
//				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
//				this_timestamps.position_target_global_int &&
//				this_timestamps.highres_imu                &&
//				this_timestamps.attitude                   &&
				this_timestamps.sys_status
				;

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_setpoint()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	mavlink_set_position_target_local_ned_t sp = current_setpoint;

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;


	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
    {
		#ifdef DEBUG
            debug_print("\nWARNING: could not send POSITION_TARGET_LOCAL_NED");
        #endif
    }
		//else
			//printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
        #ifdef DEBUG
            debug_print("\nENABLE OFFBOARD MODE");
        #endif
            
		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
            #ifdef DEBUG
                debug_print("\nError: off-board mode not set, could not write message");
            #endif
			
			//throw EXIT_FAILURE;
		}

		#ifdef DEBUG
            debug_print("\n");
        #endif
		

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			
            #ifdef DEBUG
                debug_print("\nError: off-board mode not set, could not write message");
            #endif
			//throw EXIT_FAILURE;
		}

		#ifdef DEBUG
            debug_print("\n");
        #endif
	} // end: if offboard_status

}
 
// ------------------------------------------------------------------------------
//   Arm
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
arm_disarm( bool flag )
{
	if(flag)
	{
		debug_print("\nARM ROTORS");
	}
	else
	{
		debug_print("\nDISARM ROTORS");
	}

	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = true;
	com.param1           = (float) flag;
	com.param2           = 21196;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;

    #ifdef DEBUG
        debug_print("\n toggle_offboard_control target system id=%d", system_id) ;
        debug_print("\n toggle_offboard_control target autopilot_id=%d", autopilot_id) ;
    #endif
		
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;//Use Guided Mode and the commands can be issued via Mavlink from an external source, such as a companion computer.
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;
 
	// --------------------------------------------------------------------------
	//   CHECK PORT
	// --------------------------------------------------------------------------

	if ( !port->is_running() ) // PORT_OPEN
	{
        #ifdef DEBUG
            debug_print("ERROR: port not open\n");
        #endif

		throw 1;
	}

  
	// --------------------------------------------------------------------------
	//   READ THREAD

	// --------------------------------------------------------------------------


	#ifdef DEBUG
        debug_print("\nSTART READ THREAD ok");
    #endif
            
	
	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result )
	{
        #ifdef DEBUG
            debug_print("\nThrow");
        #endif
		
	throw result;
	}


#ifdef DEBUG
        debug_print("\nSTART READ THREAD");

		// now we're reading messages
		debug_print("\n");


		// --------------------------------------------------------------------------
		//   CHECK FOR MESSAGES
		// --------------------------------------------------------------------------

		debug_print("\nCHECK FOR MESSAGES");

		debug_print("\nCurrent Message sysid, %d:", current_messages.sysid);

#endif

		

	while ( not current_messages.sysid )
	{

        #ifdef DEBUG
            debug_print("\ntime_to_exit:%d", time_to_exit) ;
        #endif

		
		if ( time_to_exit )
		{
			return;
		}
	
        #ifdef DEBUG
            debug_print("\nGo to sleep") ;
        #endif
		
		usleep(500000); // check at 2Hz
	}

	
	#ifdef DEBUG
            debug_print("\nFound\n");
            // now we know autopilot is sending messages
    #endif
            
	

	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
        
        #ifdef DEBUG
           debug_print("\nGOT VEHICLE SYSTEM ID: %i\n", system_id );
        #endif
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		
		#ifdef DEBUG
            debug_print("\nGOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		#endif
	}


	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

	// Wait for initial position ned
	while ( not ( current_messages.time_stamps.local_position_ned &&
				  current_messages.time_stamps.attitude            )  )
	{
		if ( time_to_exit )
			return;
		usleep(500000);
	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

    #ifdef DEBUG
        debug_print("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
		debug_print("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
		debug_print("\n");
    #endif
	
	// we need this before starting the write thread


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	

    #ifdef DEBUG
        debug_print("\nSTART WRITE THREAD");
    #endif
        
	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	
    #ifdef DEBUG
       debug_print("\n");
    #endif

	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	#ifdef DEBUG
       debug_print("\nCLOSE THREADS");
    #endif
	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	#ifdef DEBUG
       debug_print("\n");
    #endif
	// still need to close the port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
    #ifdef DEBUG
       debug_print("\nread thread already running");
    #endif
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
    #ifdef DEBUG
       debug_print("\nwrite thread already running");
    #endif
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) 
    {
    #ifdef DEBUG
       debug_print("\nWarning, could not stop autopilot interface error code=%d", error);
    #endif
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
	current_setpoint = sp;

	// write a message and signal writing
	//write_setpoint();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
		//write_setpoint();
	}

	// signal end
	writing_status = false;

	return;

}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

    #ifdef DEBUG
	debug_print("\nstart_autopilot_interface_read_thread"); 
    #endif
	
    // run the object's read thread
	autopilot_interface->start_read_thread();

		
	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}



