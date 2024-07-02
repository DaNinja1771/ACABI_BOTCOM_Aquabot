#ifndef BOTCOM_PARAMETERS_HPP
#define BOTCOM_PARAMETERS_HPP

#include "stdio.h"
#include "stdint.h"
#include "inttypes.h"

#include "Arduino.h"

#include "BOTCOM_multi_core.hpp"

// Drone ID Info
#define RADAGON
// #define RADAHN
// #define MARGIT
// #define MALENIA

#ifdef RADAGON
#define MODEM_6DIG          "RBD-LCX" 
#define MODEM_SN            218439
const int64_t MODEM_IMEI =  301434060056540;
#endif

#ifdef RADAHN
#define MODEM_6DIG          "FBD-XXT"
#define MODEM_SN            218454
const int64_t MODEM_IMEI =  301434060405780;
#endif
// BROKEN ONE
#ifdef MARGIT
#define MODEM_6DIG          "FPK-KVW"
#define MODEM_SN            217071
const int64_t MODEM_IMEI =  300434068396850;
#endif

#ifdef MALENIA
#define MODEM_6DIG          0
#define MODEM_SN            0
const int64_t MODEM_IMEI =  0;
#endif


// Main Program Timings

// Main Loop Runs at 1Hz 1000 ms = 1 sec
#define MAIN_LOOP_PERIOD        1000    
// Navigation Loop runs at 10Hz 100 ms = .1 sec
#define MINOR_LOOP_NAV_PERIOD   100     

// Pin Defines

// Sensors
#define SENSORS_PIN_PH              39
#define SENSORS_PIN_SALINITY        34
#define SENSORS_PIN_WIND_DIRECTION  36
#define SENSORS_PIN_WIND_SPEED      23


// OneWire Bus
#define ONEWIRE_PIN_BUS     32

// Motor Drivers
#define MOTOR_0_PIN_RIGHT_A1    33       
#define MOTOR_1_PIN_RIGHT_A2    25
#define MOTOR_2_PIN_LEFT_A1     26
#define MOTOR_3_PIN_LEFT_A2     27

// I2C Bus
#define I2C_PIN_SCL     22
#define I2C_PIN_SDA     21

// GPS
#define GPS_UART_PIN_RX     17
#define GPS_UART_PIN_TX     16

// Satellite Rockblock 9603
#define SAT_UART_PIN_RX     18
#define SAT_UART_PIN_TX     19

// Motors/Control
#define PWM_MAX                 245
#define MOTOR_PWM_MAX           235
#define BOUNDED_DEGREES         70
#define MOTOR_DUTY_CYCLE        50
#define MOTOR_PWM_BASE          160
#define MOTOR_PWM_OFF           0
#define MOTOR_PWM_BASE_CHANGE   (PWM_MAX - MOTOR_PWM_MAX)
#define PROXIMITY_THRESHOLD     5   // 5 meters
#define HEADING_TOLERANCE       5   // degrees
#define MOTOR_TURN_PWM          190
#define MOTOR_PWM_CAL           200

// Planner
#define MAX_WAYPOINTS           64

// sensor specific defines in BOTCOM_sensors.hpp
// network specific defines in network.hpp

// UARTs
// Reserved for system use DO NOT use
// #define UART_CONNECTION_0   0
// #define SYSTEM_UART         UART_CONNECTON_0 
// Reserved for GPS
#define UART_CONNECTION_1   1
#define GNSS_UART           UART_CONNECTION_1
// Reserved for satellite
#define UART_CONNECTION_2   2
#define SATELLITE_UART      UART_CONNECTION_2

// Compass
#define COMPASS_OFFSET      180         // offset compass direction by 180 so north is 0
#define COMPASS_TIMEOUT     300000      // (5 minutes)
#define COMPASS_COARSE_CALIBRATION_TIME  10000   //ms 10 sec
// Magnetic declination value only works in Tucson from https://www.magnetic-declination.com/
const double magnetic_declination_degrees = 9.0 + 14.0/60.0; 

// GPS
#define GPS_BAUD_RATE       9600
#define GPS_TIMEOUT         300000      // (5 minutes)
// Used to output year in abreviated format ex: 3/12/24
#define GPS_YEAR_ADJUST     2000

// Satellite
#define SAT_MODEM_BAUD_RATE 19200
#define SAT_MODEM_TIMEOUT   300000      // (5 minutes)

// 45 seems to work, 60 seems too long
#define NETWORK_FRACTIONAL_LOOP_PERIOD  20
#define RECEIVE_DATA_BUFFER_SIZE        320
#define REQUEST_MAX_WAYPOINTS           10

#define REQUEST_SESSION_ID_POS      0   // 4 byte
#define REQUEST_PAIR_ID_POS         4   // 4 byte
#define REQUEST_TYPE_POS            8   // 1 byte
#define REQUEST_LOOP_WAYPOINTS      9   // 1 byte
#define REQUEST_WAYPOINT_COUNT_POS  10  // 1 byte
#define REQUEST_WAYPOINTS_START_POS 11  // waypoint_count * 16 bytes

#define RESPONSE_SESSION_ID_POS         0   // 4 bytes
#define RESPONSE_PAIR_ID_POS            4   // 4 bytes
#define RESPONSE_GPS_IS_WORKING_POS     8   // 1 byte
#define RESPONSE_COMPASS_IS_WORKING_POS 9   // 1 byte
#define RESPONSE_LATITUDE_POS           10  // 8 bytes
#define RESPONSE_LONGITUDE_POS          18  // 8 bytes
#define RESPONSE_HEADING_POS            26  // 8 bytes
#define RESPONSE_SECOND_POS             34  // 1 byte
#define RESPONSE_MINUTE_POS             35  // 1 byte
#define RESPONSE_HOUR_POS               36  // 1 byte
#define RESPONSE_DAY_POS                37  // 1 byte
#define RESPONSE_MONTH_POS              38  // 1 byte
#define RESPONSE_YEAR_POS               39  // 1 byte
#define RESPONSE_STATE_POS              40  // 1 byte
#define RESPONSE_WAYPOINT_INDEX_POS     41  // 1 byte
#define RESPONSE_TEMPERATURE_POS        42  // 4 byte
#define RESPONSE_PH_POS                 46  // 4 byte
#define RESPONSE_SALINITY_POS           50  // 4 byte
#define RESPONSE_WIND_SPEED_POS         54  // 4 byte
#define RESPONSE_WIND_DIRECTION_POS     58  // 4 byte
#define RESPONSE_BATTERY_CHARGE_POS     62  // 4 byte
#define RESPONSE_IMEI                   66  // 8 byte
#define RESPONSE_SIZE                   74  // Total size + 1 byte

// remove for final


// Common Data Structs ////////////////////////////////////////////////////////////
enum class RequestType {
    Ping,
    Start,
    Stop,
    none
};// End Request Type Enum Class

// MissionStates enum
enum class MissionState {
    Initialized,
    Error_Paused,
    Satellite_Paused,
    Started,
    Stopped  
};// End MissionState Enum Class

enum class NetworkState {
    Initialized,
    PreparingSession,
    SendingMessage,
    EndingSession,
    SatelliteNetworkError,
    ModemNotWorking
};

struct GeoVec2 {
    double latitude = 0;
    double longitude = 0;
};// End GeoVec2 Struct

struct CompassData {

    bool is_updated;
    double heading;
    double heading_error;
    float offset_x;
    float offset_y;
    float offset_z;
    float scale_x;
    float scale_y;
    float scale_z;

};

struct GPSData {

    bool is_updated;
    bool location_updated;
    bool date_updated;
    bool time_updated;
    bool satellite_updated;
    uint32_t satellite_in_view;
    uint8_t day;
    uint8_t month;
    uint8_t year;               // Year, subtract 2000 from actual year
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    double latitude;
    double longitude;
    
};

struct EstimatorData {

    CompassData compass_data;
    GPSData gps_data;

    double next_waypoint_distance;
    double next_waypoint_desired_heading;

};// End EstimatorData Struct


struct PlannerData {

    RequestType current_mission_request_type;
    uint32_t session_id;
    uint32_t pair_id;
    MissionState state;
    GeoVec2 current_waypoint;
    GeoVec2 geo_waypoints[MAX_WAYPOINTS];
    uint8_t geo_waypoint_index;
    uint8_t number_of_geo_waypoints;
    bool loop_waypoints;
    
};// End PlannerData_Stuct

struct SensorData {

    float temperature;      // Temperature in degrees C
    float pH;               // pH reading
    float salinity;         // Salinity in ppm
    float windSpeed;        // Wind Speed in mph
    float windDirection;    // Wind Direction in Degrees where 0,360 = North
    float battery_charge;   // Battery Charge in Percentage of Charge Remaining
}; // End SensorData Struct

struct NetworkData{
    PlannerData planner_data;
    EstimatorData estimator_data;
    SensorData sensor_data;

    NetworkState network_state;

    uint8_t send_data_buffer[RESPONSE_SIZE];
    uint8_t receive_data_buffer[RECEIVE_DATA_BUFFER_SIZE];

    size_t receive_data_num_bytes;

    bool satellite_working;

    
    bool processed_receive_data;
    bool new_message_received;

    bool update_send_time;
    bool update_receive_time;

    int send_receive_code;

    int signal_quality;
    uint8_t network_delay_loops_remaining;

};
///////////////////////////////////////////////////////////////



#endif