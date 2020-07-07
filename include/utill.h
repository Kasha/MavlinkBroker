#ifndef COLUGO_UTILL_H_
#define COLUGO_UTILL_H_

#include <generic_port.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions
#include <queue>
#include <memory>
#include <queue>
#include <memory>
#include <string>
using namespace std ;

#define DEBUG

#ifdef DEBUG
#define DEBUG_TEST 1
#else
#define DEBUG_TEST 0
#endif


#define debug_print(fmt, ...) \
            do { if (DEBUG_TEST) fprintf(stderr, fmt, ## __VA_ARGS__); } while (0)


namespace ColugoBrokerModule
{   
enum COLUGO_BROKER_ERROR_ID
{
    COLUGO_BROKER_ERROR_NONE = 10000,
    COLUGO_BROKER_ERROR_GENERAL = 11000,
} ;

class LinkMessageBase
{
public:
    LinkMessageBase() {} ;
    virtual ~LinkMessageBase() {};
    virtual bool Push( const char* buffer_message ) {return true ;}
    virtual bool Push( const mavlink_message_t& message ){return true ;}
    virtual mavlink_message_t Pop () = 0 ;
} ;
}
#include <colugo_companion_computer.h>
#include <colugo_broker.h>
using namespace ColugoBrokerModule ;

#include <ardupilotmega/mavlink.h>
// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Time_Stamps {
    Time_Stamps()
    {
        reset_timestamps();
    }

    uint64_t heartbeat;
    uint64_t sys_status;
    uint64_t battery_status;
    uint64_t radio_status;
    uint64_t local_position_ned;
    uint64_t global_position_int;
    uint64_t position_target_local_ned;
    uint64_t position_target_global_int;
    uint64_t highres_imu;
    uint64_t attitude;

    void
    reset_timestamps()
    {
        heartbeat = 0;
        sys_status = 0;
        battery_status = 0;
        radio_status = 0;
        local_position_ned = 0;
        global_position_int = 0;
        position_target_local_ned = 0;
        position_target_global_int = 0;
        highres_imu = 0;
        attitude = 0;
    }

};

// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {

    int sysid;
    int compid;

    // Heartbeat
    mavlink_heartbeat_t heartbeat;

    // System Status
    mavlink_sys_status_t sys_status;

    // Battery Status
    mavlink_battery_status_t battery_status;

    // Radio Status
    mavlink_radio_status_t radio_status;

    // Local Position
    mavlink_local_position_ned_t local_position_ned;

    // Global Position
    mavlink_global_position_int_t global_position_int;

    // Local Position Target
    mavlink_position_target_local_ned_t position_target_local_ned;

    // Global Position Target
    mavlink_position_target_global_int_t position_target_global_int;

    // HiRes IMU
    mavlink_highres_imu_t highres_imu;

    // Attitude
    mavlink_attitude_t attitude;

    // System Parameters?


    // Time Stamps
    Time_Stamps time_stamps;

    void
    reset_timestamps()
    {
        time_stamps.reset_timestamps();
    }

};
#endif





