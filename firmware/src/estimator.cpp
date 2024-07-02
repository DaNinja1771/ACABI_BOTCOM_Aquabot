#include "estimator.hpp"

Estimator::Estimator() : gps_serial(GNSS_UART) {
    
}

 void Estimator::setup(EstimatorData &estimator_data) {

    setup_gps(estimator_data.gps_data);
    setup_compass(estimator_data.compass_data);
}

void Estimator::setup_gps(GPSData &gps_data){

    gps_serial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_UART_PIN_RX, GPS_UART_PIN_TX);
    delay(400);

    if (gps_serial.available() > 0){
        gps_try_reconnect();
    }
    
}

void Estimator::setup_compass(CompassData &compass_data){

    compass.init();
    compass.setCalibrationOffsets(0.0, 0.0, 0.0);
    compass.setCalibrationScales(1.0, 1.0, 1.0);

}

void Estimator::compass_iteration(CompassData &compass_data) {

    static double past_estimator_data_heading = 0;

    compass.read();

    compass_data.heading = atan2( compass.getY(), compass.getX() ) * 180.0 / PI;
    compass_data.heading += magnetic_declination_degrees;
    compass_data.heading -= ((COMPASS_OFFSET));
    wrap_heading_180(compass_data.heading);

    // compass reading stops changing if compass is not working
    compass_data.is_updated = past_estimator_data_heading != compass_data.heading;
    past_estimator_data_heading = compass_data.heading;
  
}

void Estimator::gps_iteration(GPSData &gps_data) {

    if (gps_serial.available()){
        
        gps_update_data(gps_data);

    }
    else{
        gps_data.is_updated = false;
        
    }
    
    if (!gps_data.is_updated) {
        
        gps_try_reconnect();
    }

    return;
}

void Estimator::compass_calibration_auto(Controller &controller, EstimatorData &estimator_data){

    controller.controller_command(MovementCommand::COMPASS_CALIBRATE);
    uint32_t compass_cal_start = millis();
    
    while((millis() - compass_cal_start) < COMPASS_COARSE_CALIBRATION_TIME){

        compass_calibration_coarse(estimator_data);
    }
    controller.controller_command(MovementCommand::STOP);

    compass_calibration_adjust();

}

void Estimator::compass_calibration_adjust(){

    compass.setCalibrationOffsets(cv.offset_x, cv.offset_y, cv.offset_z);
    compass.setCalibrationScales(cv.scale_x, cv.scale_y, cv.scale_z);

}

void Estimator::compass_calibration_coarse(EstimatorData &estimator_data){
    compass.read();

    cv.compass_min_x = (compass.getX() < cv.compass_min_x) ? compass.getX() : cv.compass_min_x;
    cv.compass_max_x = (compass.getX() > cv.compass_max_x) ? compass.getX() : cv.compass_max_x;

    cv.compass_min_y = (compass.getY() < cv.compass_min_y) ? compass.getY() : cv.compass_min_y;
    cv.compass_max_y = (compass.getY() > cv.compass_max_y) ? compass.getY() : cv.compass_max_y;
    
    cv.compass_min_z = (compass.getZ() < cv.compass_min_z) ? compass.getZ() : cv.compass_min_z;
    cv.compass_max_z = (compass.getZ() > cv.compass_max_z) ? compass.getZ() : cv.compass_max_z;


    cv.offset_x = (cv.compass_min_x + cv.compass_max_x) / 2;
    cv.offset_y = (cv.compass_min_y + cv.compass_max_y) / 2;
    cv.offset_z = (cv.compass_min_z + cv.compass_max_z) / 2;

    cv.scale_x = (cv.compass_max_x - cv.compass_min_x) / 2;
    cv.scale_y = (cv.compass_max_y - cv.compass_min_y) / 2;
    cv.scale_z = (cv.compass_max_z - cv.compass_min_z) / 2;


    cv.avg_scale = (cv.scale_x + cv.scale_y + cv.scale_z) / 3;

    cv.scale_x = cv.avg_scale / cv.scale_x;
    cv.scale_y = cv.avg_scale / cv.scale_y;
    cv.scale_z = cv.avg_scale / cv.scale_z;

    estimator_data.compass_data.offset_x = cv.offset_x;
    estimator_data.compass_data.offset_y = cv.offset_y;
    estimator_data.compass_data.offset_z = cv.offset_z;

    estimator_data.compass_data.scale_x = cv.scale_x;
    estimator_data.compass_data.scale_y = cv.scale_y;
    estimator_data.compass_data.scale_z = cv.scale_z;

}

void Estimator::gps_try_reconnect(){

    gps_serial.end();
    delay(400);
    gps_serial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_UART_PIN_RX, GPS_UART_PIN_TX);

}

void Estimator::gps_decode_data(){

    unsigned long time_start = millis();
    while (gps_serial.available() > 0 && (millis() - time_start < 200)){
        char c = gps_serial.read();
        tiny_gps.encode(c);
    }
}

void Estimator::gps_update_data(GPSData &gps_data){

    // used to get around the date issues
    // for a 1 hour period the date is guarenteed to update at least once
    static uint8_t temp_day = 0;
    static uint8_t temp_month = 0;
    static uint8_t temp_year = 0;

    gps_decode_data();

    gps_data.satellite_updated = tiny_gps.satellites.isUpdated();
    gps_data.location_updated = tiny_gps.location.isUpdated();
    gps_data.time_updated = tiny_gps.time.isUpdated();
    gps_data.date_updated = tiny_gps.date.isUpdated();

    if (gps_data.satellite_updated){
        gps_data.satellite_in_view = tiny_gps.satellites.value();
    }
    else{
        gps_data.satellite_in_view = -1;
    }


    if (gps_data.location_updated && gps_data.time_updated){

        gps_data.is_updated = true;

        if (tiny_gps.date.isUpdated()){
            temp_day = tiny_gps.date.day();
            temp_month = tiny_gps.date.month();
            temp_year = tiny_gps.date.year() - GPS_YEAR_ADJUST;
        }

        gps_data.latitude     = tiny_gps.location.lat();
        gps_data.longitude    = tiny_gps.location.lng();
        gps_data.hour         = tiny_gps.time.hour();
        gps_data.minute       = tiny_gps.time.minute();
        gps_data.second       = tiny_gps.time.second();
        gps_data.day          = temp_day;
        gps_data.month        = temp_month;
        gps_data.year         = temp_year; 
    }
    else{
        gps_data.is_updated = false;
    }

}