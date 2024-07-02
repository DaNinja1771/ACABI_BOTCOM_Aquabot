#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "BOTCOM_parameters.hpp"
#include "util_functions.hpp"
#include "controller.hpp"

#include <TinyGPS++.h>
#include <QMC5883LCompass.h>
#include <Wire.h>

struct compass_vars{
    float compass_min_x = 0;
    float compass_max_x = 0;

    float compass_min_y = 0;
    float compass_max_y = 0;
    
    float compass_min_z = 0;
    float compass_max_z = 0;

    float offset_x = 0;
    float offset_y = 0;
    float offset_z = 0;

    float avg_scale = 0;
    
    float scale_x = 0;
    float scale_y = 0;
    float scale_z = 0;
};

class Estimator {
public:
    Estimator();

    void setup(EstimatorData &estimator_data);
    void setup_gps(GPSData &gps_data);
    void setup_compass(CompassData &compass_data);

    void gps_iteration(GPSData &gps_data);
    void compass_iteration(CompassData &compass_data);

    void compass_calibration_auto(Controller &controller, EstimatorData &estiator_data);

    uint8_t day_1;
    uint8_t month_1;
    uint8_t year_1;
      
private:
    TinyGPSPlus tiny_gps;
    QMC5883LCompass compass;
    compass_vars cv;
    HardwareSerial gps_serial;

    void compass_calibration_coarse(EstimatorData &estimator_data);
    void compass_calibration_adjust(); 

    void gps_update_data(GPSData &gps_data);
    void gps_decode_data();

    void gps_try_reconnect();

};// End Estimator Class

#endif // ESTIMATOR_H