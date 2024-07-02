#include "util_functions.hpp"
#include "BOTCOM_parameters.hpp"

const char* request_type_names[4] = {"Ping", "Start", "Stop", "None"};
const char* mission_state_type_names[4] = {"Initialized", "Error Paused", "Started", "Stopped"};
const char* network_state_names[6] = {"Initialized","PreparingSession", "SendingMessage", "EndingSession", "SatelliteNetworkError", "ModemNotWorking"};
const char* send_receive_codes[11] = {"ISBD_SUCCESS [NO] Data", "ISBD_SUCCESS [YES] Data", "ISBD_SENDRECEIVE_TIMEOUT", "SendReceive Not Run", "ISBD_IS_ASLEEP", "ISBD_CANCELLED", "ISBD_SBDIX_FATAL_ERROR", "ISBD_PROTOCOL_ERROR", "ISBD_MSG_TOO_LONG", "ISBD_REENTRANT", "Default Error [Something Horrible]"};
debug_data debug_var;

void wrap_heading_180(double &heading){
    while (heading > 180){
        heading -= 360;
    }
    while (heading < -180) {
        heading += 360;
    }
}

void set_mission_state_new(PlannerData &planner_data, MissionState new_state){
    // planner_data.prev_state = planner_data.state;
    planner_data.state = new_state;
}

void print_debug_dump_general(EstimatorData &estimator_data, PlannerData &planner_data, SensorData &sensor_data, NetworkData &network_data){    

    if (estimator_data.compass_data.is_updated){
        debug_logln(1, "Heading: %lf", estimator_data.compass_data.heading);
        debug_logln(1, "Heading Difference: %lf", estimator_data.compass_data.heading_error);
        debug_logln(1, "Offset X: %f", estimator_data.compass_data.offset_x);
        debug_logln(1, "Offset Y: %f", estimator_data.compass_data.offset_y);
        debug_logln(1, "Offset Z: %f", estimator_data.compass_data.offset_z);
        debug_logln(1, "Scale X: %f", estimator_data.compass_data.scale_x);
        debug_logln(1, "Scale Y: %f", estimator_data.compass_data.scale_y);
        debug_logln(1, "Scale Z: %f", estimator_data.compass_data.scale_z);
        
    }
    else{
        debug_logln(1, "Heading: [Error]");
        debug_logln(1, "Heading Difference: [Error]");
    }
    
    
    // gps
    if (estimator_data.gps_data.satellite_in_view < 0){
        debug_logln("Satellites in View: [Error]");
    }
    else{
        debug_logln("Satellites in View: %d", estimator_data.gps_data.satellite_in_view);
    }

    if (estimator_data.gps_data.location_updated == 1){
        debug_logln(1, "Latitude: %lf", estimator_data.gps_data.latitude);
        debug_logln(1, "Longitude: %lf", estimator_data.gps_data.longitude);
    }
    else{
        debug_logln(1, "Latitude: [Error]");
        debug_logln(1, "Longitude: [Error]");
    }
    // debug_logln(1, "GPS Time Status: ", debug_var.gps_time_working == 1 ? "[Working]":"[Error]");
    if(estimator_data.gps_data.time_updated == 1){
        debug_logln(1, "Time: %d:%d:%d",    estimator_data.gps_data.hour,
                                            estimator_data.gps_data.minute,
                                            estimator_data.gps_data.second
                                            );
    }
    else{
        debug_logln(1, "Time: [Error]");
    }
    // debug_logln(1, "GPS Date Status: ", debug_var.gps_date_working == 1 ? "[Working]":"[Error]");
    if(estimator_data.gps_data.date_updated == 1){
        debug_logln(1, "Date: %d/%d/%d",    estimator_data.gps_data.day,
                                            estimator_data.gps_data.month,
                                            estimator_data.gps_data.year
                                            );
    }
    else{
        debug_logln(1, "Date: [Error]");
    }

    // debug_logln("Planner Module Status:");

    if (planner_data.state != MissionState::Initialized){
        
        debug_logln(1, "Current Session ID: %u", planner_data.session_id);
        debug_logln(1, "Current Pair ID: %u", planner_data.pair_id);
        debug_logln(1, "Current Mission State: %d", static_cast<uint8_t>(planner_data.state));
        debug_logln(1, "Current Mission Request Type: %d", static_cast<uint8_t>(planner_data.current_mission_request_type));
        debug_logln(1, "Next Waypoint: %d out of %d", planner_data.geo_waypoint_index, planner_data.number_of_geo_waypoints);
        debug_logln(2, "Next Waypoint Latitude: %lf", planner_data.current_waypoint.latitude);
        debug_logln(2, "Next Waypoint Longitude: %lf", planner_data.current_waypoint.longitude);
        debug_log("Looping Waypoints: ");
        if(planner_data.loop_waypoints == 1){
            debug_logln("[True]");
        }
        else{
            debug_logln("[False]");
        }
        debug_logln(2, "Distance to Next Waypoint: %lf", estimator_data.next_waypoint_distance);
        debug_logln(2, "Desired Heading: %lf", estimator_data.next_waypoint_desired_heading);
        debug_log("List of Waypoints: "); 
        for (int i = 0; i < planner_data.number_of_geo_waypoints; i++){
            debug_log("{ %lf, ", planner_data.geo_waypoints[i].latitude);
            debug_log("%lf }, ", planner_data.geo_waypoints[i].longitude);
        }
        debug_log("\n");
    }
    else{
        debug_logln(1, "Current Session ID: XXXX");
        debug_logln(1, "Current Pair ID: XXXX");
        debug_logln(1, "Current Mission State: XXXX");
        debug_logln(1, "Current Mission Request Type: XXXX");
        debug_logln(1, "Heading to Waypoint: XXXX");
        debug_logln(2, "Heading to Latitude: XXXX");
        debug_logln(2, "Heading to Longitude: XXXX");
        debug_logln(1, "Looping Waypoints: XXXX");
        debug_logln(2, "Distance to next Waypoint: XXXX");
        debug_logln(2, "Desired Heading: XXXX");
        debug_logln("List of Waypoints: XXXX"); 
    }
    
    // debug_logln("Sensor Module Status:");
    debug_logln(1, "Temperature: %f", sensor_data.temperature);
    debug_logln(1, "pH: %f", sensor_data.pH);
    debug_logln(1, "Salinity: %f", sensor_data.salinity);
    debug_logln(1, "Wind Speed: %f", sensor_data.windSpeed);
    debug_logln(1, "Wind Direction: %f", sensor_data.windDirection);
    debug_logln(1, "Battery Charge: %f", sensor_data.battery_charge);

    // debug_logln("Network Module Status:");

    debug_log(1, "Modem Activity: ");
    if (network_data.satellite_working == 1){
        debug_logln("[Working]");
    }
    else{
        debug_logln("[Error]");
    }
    
    if (network_data.update_receive_time){
        debug_logln("update receive time:");
        network_data.update_receive_time = false;

        debug_var.print_receive_arr = true;
    }


    if (debug_var.print_receive_arr){

        debug_logln(0, "Session Results: %s",       send_receive_codes[network_data.send_receive_code]);
        
        debug_logln("Receive Session ID: %u\n",     *(uint32_t *)(network_data.receive_data_buffer + REQUEST_SESSION_ID_POS));
        debug_logln("Receive Pair ID: %u\n",        *(uint32_t *)(network_data.receive_data_buffer + REQUEST_PAIR_ID_POS));
        debug_logln("Receive Request Type: %u\n",   *(uint8_t *)(network_data.receive_data_buffer + REQUEST_TYPE_POS));
        debug_logln("Receive Loop Waypoints: %u\n", *(uint8_t *)(network_data.receive_data_buffer + REQUEST_LOOP_WAYPOINTS));
        debug_logln("Receive Waypoint Count: %u\n", *(uint8_t *)(network_data.receive_data_buffer + REQUEST_WAYPOINT_COUNT_POS));

        // Now read the waypoints
        const unsigned char *waypointData = network_data.receive_data_buffer + REQUEST_WAYPOINTS_START_POS;

        debug_log("Received Waypoints: ");

        for (int i = 0; i < *(uint8_t *)(network_data.receive_data_buffer + REQUEST_WAYPOINT_COUNT_POS); i++) {
            GeoVec2 wp = *(GeoVec2 *)(waypointData + i * sizeof(GeoVec2));
            debug_log("{ %lf, ", wp.latitude);
            debug_log("%lf }, ", wp.longitude);
        }
        debug_log("\n");
    }

    if (network_data.update_send_time){
        debug_logln("update send time:");
        network_data.update_send_time = false;
        debug_var.print_send_arr = true;
    }
    if (debug_var.print_send_arr){
        debug_logln("Send Session ID: %u",      *(uint32_t *)(network_data.send_data_buffer + RESPONSE_SESSION_ID_POS));
        debug_logln("Send Pair ID: %u",         *(uint32_t *)(network_data.send_data_buffer + RESPONSE_PAIR_ID_POS));
        debug_logln("Send GPS Working: %d",     *(uint8_t *)(network_data.send_data_buffer + RESPONSE_LATITUDE_POS));
        debug_logln("Send Longitude: %lf",      *(double *)(network_data.send_data_buffer + RESPONSE_LONGITUDE_POS));
        debug_logln("Send Heading: %lf",        *(double *)(network_data.send_data_buffer + RESPONSE_HEADING_POS));
        debug_logln("Send Second: %d",          *(uint8_t *)(network_data.send_data_buffer + RESPONSE_SECOND_POS));
        debug_logln("Send Minute: %d",          *(uint8_t *)(network_data.send_data_buffer + RESPONSE_MINUTE_POS));
        debug_logln("Send Hour: %d",            *(uint8_t *)(network_data.send_data_buffer + RESPONSE_HOUR_POS));
        debug_logln("Send Day: %d",             *(uint8_t *)(network_data.send_data_buffer + RESPONSE_DAY_POS));
        debug_logln("Send Month: %d",           *(uint8_t *)(network_data.send_data_buffer + RESPONSE_MONTH_POS));
        debug_logln("Send Year: %d",            *(uint8_t *)(network_data.send_data_buffer + RESPONSE_YEAR_POS));
        debug_logln("Send State: %d",           *(uint8_t *)(network_data.send_data_buffer + RESPONSE_STATE_POS));
        debug_logln("Send Waypoint Index: %d",  *(uint8_t *)(network_data.send_data_buffer + RESPONSE_WAYPOINT_INDEX_POS));
        debug_logln("Send Temperature: %f",     *(float *)(network_data.send_data_buffer + RESPONSE_TEMPERATURE_POS));
        debug_logln("Send pH: %f",              *(float *)(network_data.send_data_buffer + RESPONSE_PH_POS));
        debug_logln("Send Salinity: %f",        *(float *)(network_data.send_data_buffer + RESPONSE_SALINITY_POS));
        debug_logln("Send Wind Speed: %f",      *(float *)(network_data.send_data_buffer + RESPONSE_WIND_SPEED_POS));
        debug_logln("Send Wind Direction: %f",  *(float *)(network_data.send_data_buffer + RESPONSE_WIND_DIRECTION_POS));
        debug_logln("Send Battery Charge: %f",  *(float *)(network_data.send_data_buffer + RESPONSE_BATTERY_CHARGE_POS));
        debug_logln("Send IMEI: %lld",          *(long long int *)(network_data.send_data_buffer + RESPONSE_IMEI));
        
    }
    debug_logln(0, "Network State: %s",     network_state_names[static_cast<uint8_t>(network_data.network_state)]);
    debug_logln("Signal Quality: %d",       network_data.signal_quality);
    debug_logln("Loops Before Transmission: %d", NETWORK_FRACTIONAL_LOOP_PERIOD - network_data.network_delay_loops_remaining);

}

bool log_active = true;

void set_log_mode(bool _log_active){
    log_active = _log_active;
}

void debug_log(const char* format, ...) {
    if (!log_active) return;

    const char* tab = "    ";
    char newFormat[256];
    
    // Create the indentation string
    char indentation[256] = "";
    for (int i = 0; i < 0; i++) {
        strcat(indentation, tab);
    }

    // Append the original format string to the indentation
    snprintf(newFormat, sizeof(newFormat), "%s%s", indentation, format);

    char buffer[256]; // Buffer to hold the formatted string
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), newFormat, args);
    va_end(args);

    Serial.print(buffer);
}

void debug_logln(const char* format, ...) {
    if (!log_active) return;
    const char* tab = "    ";
    char newFormat[256];
    
    // Create the indentation string
    char indentation[256] = "";
    for (int i = 0; i < 0; i++) {
        strcat(indentation, tab);
    }

    // Append the original format string to the indentation
    snprintf(newFormat, sizeof(newFormat), "%s%s\n", indentation, format);

    char buffer[256]; // Buffer to hold the formatted string
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), newFormat, args);
    va_end(args);

    Serial.print(buffer);
}

void debug_log(int offset, const char* format, ...) {
    if (!log_active) return;
    const char* tab = "    ";
    char newFormat[256];
    
    // Create the indentation string
    char indentation[256] = "";
    for (int i = 0; i < offset; i++) {
        strcat(indentation, tab);
    }

    // Append the original format string to the indentation
    snprintf(newFormat, sizeof(newFormat), "%s%s", indentation, format);

    char buffer[256]; // Buffer to hold the formatted string
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), newFormat, args);
    va_end(args);

    Serial.print(buffer);
}

void debug_logln(int offset, const char* format, ...) {
    if (!log_active) return;
    const char* tab = "    ";
    char newFormat[256];
    
    // Create the indentation string
    char indentation[256] = "";
    for (int i = 0; i < offset; i++) {
        strcat(indentation, tab);
    }

    // Append the original format string to the indentation
    snprintf(newFormat, sizeof(newFormat), "%s%s\n", indentation, format);

    char buffer[256]; // Buffer to hold the formatted string
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), newFormat, args);
    va_end(args);

    Serial.print(buffer);
}
