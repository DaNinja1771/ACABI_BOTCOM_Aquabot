#ifndef NETWORK_HPP
#define NETWORK_HPP

#include "BOTCOM_parameters.hpp"
#include "util_functions.hpp"

#include <Arduino.h>
#include <HardwareSerial.h>
#include "IridiumSBD.h"


class Network {
public:

    // Default Constructor
    Network();
    // Sets up the object's data
    void setup();
 
    void iteration_3();

    int get_signal_quality();
    bool modem_status();

    void sync_network_data( EstimatorData &estimator_data, 
                            PlannerData &planner_data, 
                            SensorData &sensor_data,
                            NetworkData &network_data
                            );

    void processRequest_2();
    void queueResponse_2();
    NetworkState get_network_state();
    int send_receive_return;

private:

    HardwareSerial modem_serial;
    IridiumSBD modem;
    NetworkState network_state;

    NetworkData class_network_data;

    uint32_t modem_last_connected;

    bool satelite_connect_helper;

    void set_network_state(NetworkData &network_data, NetworkState new_state);

};// End Network Class



#endif // NETWORK_HPP
