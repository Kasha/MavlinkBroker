/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *           Liad Kashanovsky <liadky@gmail.com>
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
 * @file mavlink_control.cpp
 *
 * @companion app for communicating between companion apps running on  machines such as NVIDIA TX2 and Ardupilot/PX4
 *
 * This process connects an external MAVLink UART device to send and receive data with companion apps sensors data correction
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 * @Liad Kashanovsky <liadky@gmail.com>
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"

// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------

int
top ( int argc, char **argv )
{

    // --------------------------------------------------------------------------
    //   PARSE THE COMMANDS
    // --------------------------------------------------------------------------

    // Default input arguments

    char *uart_name = ( char* ) "/dev/ttyTHS2";
    int baudrate = 57600;

    bool use_udp = false;
    char *udp_ip = ( char* ) "127.0.0.1";
    int udp_port = 14540;
    bool autotakeoff = false;

    // do the parse, will throw an int if it fails
    parse_commandline ( argc, argv, uart_name, baudrate, use_udp, udp_ip, udp_port, autotakeoff );


    // --------------------------------------------------------------------------
    //   PORT and THREAD STARTUP
    // --------------------------------------------------------------------------

    /*
     * Instantiate a generic port object
     *
     * This object handles the opening and closing of the offboard computer's
     * port over which it will communicate to an autopilot.  It has
     * methods to read and write a mavlink_message_t object.  To help with read
     * and write in the context of pthreading, it gaurds port operations with a
     * pthread mutex lock. It can be a serial or an UDP port.
     *
     */
    Generic_Port *port;
    if ( use_udp ) {
        port = new UDP_Port ( udp_ip, udp_port );
    } else {
        port = new Serial_Port ( uart_name, baudrate );
    }


    /*
     * Instantiate an autopilot interface object
     *
     * This starts two threads for read and write over MAVlink. The read thread
     * listens for any MAVlink message and pushes it to the current_messages
     * attribute.  The write thread at the moment only streams a position target
     * in the local NED frame (mavlink_set_position_target_local_ned_t), which
     * is changed by using the method update_setpoint().  Sending these messages
     * are only half the requirement to get response from the autopilot, a signal
     * to enter "offboard_control" mode is sent by using the enable_offboard_control()
     * method.  Signal the exit of this mode with disable_offboard_control().  It's
     * important that one way or another this program signals offboard mode exit,
     * otherwise the vehicle will go into failsafe.
     *
     */
    Autopilot_Interface autopilot_interface ( port );

    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    port_quit         = port;
    autopilot_interface_quit = &autopilot_interface;
    signal ( SIGINT,quit_handler );

    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */
    port->start();
    autopilot_interface.start();

    // --------------------------------------------------------------------------
    //   START OFFBOARD MODE
    // --------------------------------------------------------------------------
    //Use Guided Mode and the commands can be issued via Mavlink from an external source, such as a companion computer.
    autopilot_interface.enable_offboard_control();
    usleep ( 100 ); // give some time to let it sink in



    // --------------------------------------------------------------------------
    //   STOP OFFBOARD MODE
    // --------------------------------------------------------------------------

    autopilot_interface.disable_offboard_control();

    // now pixhawk isn't listening to setpoint commands


    // --------------------------------------------------------------------------
    //   RUN COMMANDS
    // --------------------------------------------------------------------------

    /*
     * Now we can implement the algorithm we want on top of the autopilot interface
     */
    //commands(autopilot_interface, autotakeoff);


    // --------------------------------------------------------------------------
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------

    /*
     * Now that we are done we can stop the threads and close the port
     */
    autopilot_interface.stop();
    port->stop();

    delete port;

    // --------------------------------------------------------------------------
    //   DONE
    // --------------------------------------------------------------------------

    // woot!
    return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands ( Autopilot_Interface &api, bool autotakeoff )
{

    // --------------------------------------------------------------------------
    //   START OFFBOARD MODE
    // --------------------------------------------------------------------------

    //api.enable_offboard_control();
    //usleep(100); // give some time to let it sink in



    // --------------------------------------------------------------------------
    //   STOP OFFBOARD MODE
    // --------------------------------------------------------------------------

    //api.disable_offboard_control();

    // now pixhawk isn't listening to setpoint commands


    // --------------------------------------------------------------------------
    //   GET A MESSAGE
    // --------------------------------------------------------------------------
    /*printf("READ SOME MESSAGES \n");

    // copy current messages
    Mavlink_Messages messages = api.current_messages;

    // local position in ned frame
    mavlink_local_position_ned_t pos = messages.local_position_ned;
    printf("Got message LOCAL_POSITION_NED (spec: https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED)\n");
    printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

    // hires imu
    mavlink_highres_imu_t imu = messages.highres_imu;
    printf("Got message HIGHRES_IMU (spec: https://mavlink.io/en/messages/common.html#HIGHRES_IMU)\n");
    printf("    ap time:     %lu \n", imu.time_usec);
    printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
    printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
    printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
    printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
    printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
    printf("    temperature: %f C \n"       , imu.temperature );

    printf("\n");*/


    // --------------------------------------------------------------------------
    //   END OF COMMANDS
    // --------------------------------------------------------------------------

    return;

}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline ( int argc, char **argv, char *&uart_name, int &baudrate,
                    bool &use_udp, char *&udp_ip, int &udp_port, bool &autotakeoff )
{

    // string for command line usage
    const char *commandline_usage = "usage: mavlink_control [-d <devicename> -b <baudrate>] [-u <udp_ip> -p <udp_port>] [-a ]";

    // Read input arguments
    for ( int i = 1; i < argc; i++ ) { // argv[0] is "mavlink"

        // Set Debug
        //if (strcmp(argv[i], "-g") == 0 || strcmp(argv[i], "--debug") == 0) {
        //	g_bDebug = true ;
        //}

        // Help
        if ( strcmp ( argv[i], "-h" ) == 0 || strcmp ( argv[i], "--help" ) == 0 ) {
#ifdef DEBUG
            debug_print ( "%s\n",commandline_usage );
#endif
            throw EXIT_FAILURE;
        }

        // UART device ID
        if ( strcmp ( argv[i], "-d" ) == 0 || strcmp ( argv[i], "--device" ) == 0 ) {
            if ( argc > i + 1 ) {
                i++;
                uart_name = argv[i];
            } else {
#ifdef DEBUG
                debug_print ( "%s\n",commandline_usage );
#endif
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if ( strcmp ( argv[i], "-b" ) == 0 || strcmp ( argv[i], "--baud" ) == 0 ) {
            if ( argc > i + 1 ) {
                i++;
                baudrate = atoi ( argv[i] );
            } else {
#ifdef DEBUG
                debug_print ( "%s\n",commandline_usage );
#endif
                throw EXIT_FAILURE;
            }
        }

        // UDP ip
        if ( strcmp ( argv[i], "-u" ) == 0 || strcmp ( argv[i], "--udp_ip" ) == 0 ) {
            if ( argc > i + 1 ) {
                i++;
                udp_ip = argv[i];
                use_udp = true;
            } else {
#ifdef DEBUG
                debug_print ( "%s\n",commandline_usage );
#endif
                throw EXIT_FAILURE;
            }
        }

        // UDP port
        if ( strcmp ( argv[i], "-p" ) == 0 || strcmp ( argv[i], "--port" ) == 0 ) {
            if ( argc > i + 1 ) {
                i++;
                udp_port = atoi ( argv[i] );
            } else {
#ifdef DEBUG
                debug_print ( "%s\n",commandline_usage );
#endif
                throw EXIT_FAILURE;
            }
        }

        /*// Autotakeoff
        if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--autotakeoff") == 0) {
        	autotakeoff = true;
        }*/

    }
    // end: for each input argument


    // Done!
    return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler ( int sig )
{
#ifdef DEBUG
    debug_print ( "\nTERMINATING AT USER REQUEST\n" );
#endif

    // autopilot interface
    try {
        autopilot_interface_quit->handle_quit ( sig );
    } catch ( int error ) {}

    // port
    try {
        port_quit->stop();
    } catch ( int error ) {}

    // end program here
    exit ( 0 );

}

extern ColugoBrokerManager* ColugoBrokerManager::singleton_ ;
// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main ( int argc, char **argv )
{
    try {
        ColugoBrokerManager::Start();
        ColugoBrokerManager::Stop();
        int result = top ( argc,argv );
        return result;
    }

    catch ( int error ) 
    {
        switch(error)
        { 
            case COLUGO_BROKER_STOP:
            {
                //Write to db or log
                break ;
            }
        }
#ifdef DEBUG
        debug_print ( "mavlink_control threw exception %i \n" , error );
#endif
        return error;
    }

}



