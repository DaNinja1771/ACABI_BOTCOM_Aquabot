#include "network.hpp"

Network::Network() : modem_serial(SATELLITE_UART), modem(modem_serial){

}

void Network::setup() {

    set_network_state(class_network_data, NetworkState::Initialized);

    
    // modem_last_connected = millis();
    class_network_data.network_delay_loops_remaining = 0;
    satelite_connect_helper = true;


    // Satellite Mode
    // init modem serial
    modem_serial.begin(SAT_MODEM_BAUD_RATE, SERIAL_8N1, SAT_UART_PIN_RX, SAT_UART_PIN_TX);
    delay(400);
    // set modem power profile
    // The difference between these is changing timmings to account for the super capacitors charging
    // after a message
    modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);
    // modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
    // modem.setPowerProfile(IridiumSBD::CUSTOM_POWER_PROFILE);

    // Start Modem operations
    modem.begin();
    
}

// Get the network state when network functions are busy
NetworkState Network::get_network_state(){
    return class_network_data.network_state;
}

// Main workings of the network 
void Network::iteration_3(){
    // These are used for debugging purposes

    if (modem_status() != 1){
        set_network_state(class_network_data, NetworkState::ModemNotWorking);
        return;
    }


    class_network_data.update_send_time = false;
    class_network_data.update_receive_time = false;
    // class_network_data.network_state = NetworkState::PreparingSession;
    set_network_state(class_network_data, NetworkState::PreparingSession);

    // This is used for error handling
    send_receive_return = -1;

    // Creates an artificial delay of around 20 seconds to make sure Iridium is not kicking you out
    // for sending too many messages in too short of a time window. This can be updated in the 
    // parameters file
    if (class_network_data.network_delay_loops_remaining != NETWORK_FRACTIONAL_LOOP_PERIOD){
        class_network_data.network_delay_loops_remaining++;
        
        get_signal_quality();
        return;
    }
    
    class_network_data.network_delay_loops_remaining = 0;

    if (satelite_connect_helper == true){
        modem_last_connected = millis();
        satelite_connect_helper = false;
    }

    if ((millis() - modem_last_connected >= SAT_MODEM_TIMEOUT)){
        set_network_state(class_network_data, NetworkState::SatelliteNetworkError);
        // class_network_data.network_state = NetworkState::ModuleError;
        class_network_data.satellite_working = false;
    }
    else {
        class_network_data.satellite_working = true;
    }

    // If the modem is broken or not connected
    // if(modem_status() != 1){

        
    //     // check to see if the last time we connected to the satellites was 5 minutes agao
        
    //     return;
    // }
    // else{
        
    // }
    // This tells the size of the array that will receive the data
    class_network_data.receive_data_num_bytes = sizeof(class_network_data.receive_data_buffer);
    
    // get_signal_quality();
    // queue send message with the most up to date data
    queueResponse_2();
    // class_network_data.network_state = NetworkState::SendingMessage;
    set_network_state(class_network_data, NetworkState::SendingMessage);
    class_network_data.update_send_time = true;
    // This is the actual function that sends/receives data
    send_receive_return = modem.sendReceiveSBDBinary(class_network_data.send_data_buffer, RESPONSE_SIZE, class_network_data.receive_data_buffer, class_network_data.receive_data_num_bytes);
    
    // class_network_data.network_state = NetworkState::EndingSession;
    set_network_state(class_network_data, NetworkState::EndingSession);
    // this is the error handling
    switch (send_receive_return){
        // This is the case whenever a connection is made. We can still connect and not receive any data 
        // so we update the time that was last connected here 
        case ISBD_SUCCESS:
            class_network_data.network_state = NetworkState::EndingSession;
            class_network_data.update_receive_time = true;
            modem_last_connected = millis();
            debug_log("ISBD_SUCCESS ");
            if (xSemaphoreTake(processed_request_mutex, portMAX_DELAY) == pdTRUE){
                if (class_network_data.receive_data_num_bytes == 0) {
                    class_network_data.new_message_received = false;
                    class_network_data.processed_receive_data = false;
                    class_network_data.send_receive_code = 0;
                } 
                else {
                    processRequest_2();
                    class_network_data.new_message_received = true;
                    class_network_data.send_receive_code = 1;
                }
                xSemaphoreGive(processed_request_mutex);
            }
            break;
        case ISBD_SENDRECEIVE_TIMEOUT:
            class_network_data.send_receive_code = 2;
            break;
        case -1:
            class_network_data.send_receive_code = 3;
            break;
        case ISBD_IS_ASLEEP:
            class_network_data.send_receive_code = 4;
            break;
        case ISBD_CANCELLED:
            class_network_data.send_receive_code = 5;
            break;
        case ISBD_SBDIX_FATAL_ERROR:
            class_network_data.send_receive_code = 6;
            break;
        case ISBD_PROTOCOL_ERROR:
            class_network_data.send_receive_code = 7;
            break;
        case ISBD_MSG_TOO_LONG:
            class_network_data.send_receive_code = 8;
            break;
        case ISBD_REENTRANT:
            class_network_data.send_receive_code = 9;
            break;
        default:
            class_network_data.send_receive_code = 10;
            break;
    }

    // if (send_receive_return == ISBD_SUCCESS){

    // }

}

// This basically stores the data received from the message into another variable for the drone to use
void Network::processRequest_2() {
    // debug_logln("processRequest_2:");
    RequestType request_type = static_cast<RequestType>(class_network_data.receive_data_buffer[REQUEST_TYPE_POS]);

    // Extract session ID and pair ID from the network buffer
    class_network_data.planner_data.session_id = *((uint32_t*)(&class_network_data.receive_data_buffer[RESPONSE_SESSION_ID_POS]));
    class_network_data.planner_data.pair_id = *((uint32_t*)(&class_network_data.receive_data_buffer[RESPONSE_PAIR_ID_POS]));
    class_network_data.planner_data.loop_waypoints = class_network_data.receive_data_buffer[REQUEST_LOOP_WAYPOINTS];

    int current_byte = REQUEST_WAYPOINTS_START_POS;
    
    class_network_data.planner_data.current_mission_request_type = request_type;
    
    switch (request_type) {
        case RequestType::Start:
            
            
            set_mission_state_new(class_network_data.planner_data, MissionState::Started);
            
            class_network_data.planner_data.number_of_geo_waypoints = class_network_data.receive_data_buffer[REQUEST_WAYPOINT_COUNT_POS];
            class_network_data.planner_data.geo_waypoint_index = 0;

             // Process the geo_waypoints
            for (int i = 0; i < class_network_data.planner_data.number_of_geo_waypoints; i++) {
                // Deserialize latitude and longitude from the network_buffer
                memcpy(&class_network_data.planner_data.geo_waypoints[i].latitude, class_network_data.receive_data_buffer + current_byte, sizeof(double));
                current_byte += sizeof(double);
                memcpy(&class_network_data.planner_data.geo_waypoints[i].longitude, class_network_data.receive_data_buffer + current_byte, sizeof(double));
                current_byte += sizeof(double);
                
            }

            break;

        case RequestType::Stop:
            set_mission_state_new(class_network_data.planner_data, MissionState::Stopped);
            break;
        
        default:
            break;
    }
    class_network_data.processed_receive_data = true;
}

// This is the storing of the current drones data into the array used for sending
void Network::queueResponse_2(){
 
    // EstimatorData info
    memcpy(class_network_data.send_data_buffer + RESPONSE_SESSION_ID_POS, &class_network_data.planner_data.session_id, sizeof(uint32_t));
    memcpy(class_network_data.send_data_buffer + RESPONSE_PAIR_ID_POS, &class_network_data.planner_data.pair_id, sizeof(uint32_t));
    // memcpy(send_data_buffer + RESPONSE_PAIR_ID_POS, &drone_id, sizeof(uint32_t));
    
    class_network_data.send_data_buffer[RESPONSE_GPS_IS_WORKING_POS] = class_network_data.estimator_data.gps_data.is_updated ? 1 : 0;
    class_network_data.send_data_buffer[RESPONSE_COMPASS_IS_WORKING_POS] = class_network_data.estimator_data.compass_data.is_updated ? 1 : 0;
    memcpy(class_network_data.send_data_buffer + RESPONSE_LATITUDE_POS, &class_network_data.estimator_data.gps_data.latitude, sizeof(double));
    memcpy(class_network_data.send_data_buffer + RESPONSE_LONGITUDE_POS, &class_network_data.estimator_data.gps_data.longitude, sizeof(double));
    memcpy(class_network_data.send_data_buffer + RESPONSE_HEADING_POS, &class_network_data.estimator_data.compass_data.heading, sizeof(double));
    class_network_data.send_data_buffer[RESPONSE_SECOND_POS] = class_network_data.estimator_data.gps_data.second;
    class_network_data.send_data_buffer[RESPONSE_MINUTE_POS] = class_network_data.estimator_data.gps_data.minute;
    class_network_data.send_data_buffer[RESPONSE_HOUR_POS] = class_network_data.estimator_data.gps_data.hour;
    class_network_data.send_data_buffer[RESPONSE_DAY_POS] = class_network_data.estimator_data.gps_data.day;
    class_network_data.send_data_buffer[RESPONSE_MONTH_POS] = class_network_data.estimator_data.gps_data.month;
    class_network_data.send_data_buffer[RESPONSE_YEAR_POS] = class_network_data.estimator_data.gps_data.year;

    // PlannerData info
    class_network_data.send_data_buffer[RESPONSE_STATE_POS] = static_cast<uint8_t>(class_network_data.planner_data.state);
    class_network_data.send_data_buffer[RESPONSE_WAYPOINT_INDEX_POS] = class_network_data.planner_data.geo_waypoint_index;

    // SensorData info
    memcpy(class_network_data.send_data_buffer + RESPONSE_TEMPERATURE_POS, &class_network_data.sensor_data.temperature, sizeof(float));
    memcpy(class_network_data.send_data_buffer + RESPONSE_PH_POS, &class_network_data.sensor_data.pH, sizeof(float));
    memcpy(class_network_data.send_data_buffer + RESPONSE_SALINITY_POS, &class_network_data.sensor_data.salinity, sizeof(float));
    memcpy(class_network_data.send_data_buffer + RESPONSE_WIND_SPEED_POS, &class_network_data.sensor_data.windSpeed, sizeof(float));
    memcpy(class_network_data.send_data_buffer + RESPONSE_WIND_DIRECTION_POS, &class_network_data.sensor_data.windDirection, sizeof(float));
    memcpy(class_network_data.send_data_buffer + RESPONSE_BATTERY_CHARGE_POS, &class_network_data.sensor_data.battery_charge, sizeof(float));
    memcpy(class_network_data.send_data_buffer + RESPONSE_IMEI, &MODEM_IMEI, sizeof(int64_t));

}
// gets the current signal quality
int Network::get_signal_quality(){

    int qual2 = -1;

    int err = modem.getSignalQuality(qual2);
    class_network_data.signal_quality = qual2;

    switch (err){
        case ISBD_IS_ASLEEP:
            break;
        case ISBD_CANCELLED:
            break;
        case ISBD_PROTOCOL_ERROR:
            break;
        case ISBD_SUCCESS:
            break;
    }

    return qual2;
}
// checks if the modem has been disconnected or is unable to respond to messages from the ESP32
bool Network::modem_status(){

    modem_serial.println("AT");

    delay(1000);

    String response = "";

    while (modem_serial.available()){
        char c = modem_serial.read();
        response += c;
    }

    if (response.indexOf("OK") != -1){
        return 1;
    }
    
    return 0;
}
// sync the drones data with the network 
void Network::sync_network_data(EstimatorData &estimator_data, PlannerData &planner_data, SensorData &sensor_data, NetworkData &network_data){

    if(class_network_data.processed_receive_data){
        planner_data = class_network_data.planner_data;
        
    }
    else{
        class_network_data.planner_data = planner_data;
    }

    class_network_data.estimator_data = estimator_data;
    class_network_data.sensor_data = sensor_data;
    network_data = class_network_data;
}

void Network::set_network_state(NetworkData &network_data, NetworkState new_state){
    if (network_data.network_state != NetworkState::SatelliteNetworkError){
        network_data.network_state = new_state;
    }
}
