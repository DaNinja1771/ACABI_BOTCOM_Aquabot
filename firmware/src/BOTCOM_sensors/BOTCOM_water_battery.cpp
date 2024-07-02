#include "BOTCOM_sensors.hpp"

void Sensors::update_data_ph(float &ph_data) {

    uint16_t ph_avg_adc_reading = 0;
    float ph_result = 0.0;

    // gets average analog value from sensor
    ph_avg_adc_reading = average_value_analog(SENSORS_PIN_PH);

    // get the average voltage before converting into pH
    ph_result = (float)ph_avg_adc_reading * VOLTAGE_ADC_3312;

    // linear conversion formula (found using excel plot)
    ph_result = (12.5 * ph_result) - 7.2;

    if (ph_result > 14 || ph_result < 0){
        ph_data = ERROR_PH_SENSOR;
        return;
    }

    ph_data = ph_result;
}

void Sensors::update_data_salinity(float &salinity_data) {

    uint16_t tds_avg_adc_reading = 0;
    float tds_result = 0.0;

    // gets average analog value from sensor
    tds_avg_adc_reading = average_value_analog(SENSORS_PIN_SALINITY);
    // convert the analog value into digital
    tds_result = (float)tds_avg_adc_reading * VOLTAGE_ADC_3312;
    // this finds the total disolved solids value
    tds_result = ((133.42 * pow(tds_result, 3)) - (255.86 * pow(tds_result, 2)) + (857.39 * tds_result)) * SALINITY_KVALUE;
    // this corrects for temperature
    tds_result = tds_result / (1.0 + 0.02 * (sensor_class_data.temperature - 25.0));
    // provides the approximation of salinity in the solution
    tds_result = tds_result * SALINITY_FACTOR;

    if(tds_result > 1000 || tds_result < 0){
        tds_result = ERROR_TDS_SENSOR;
        return;
    }

    salinity_data = tds_result;
}

void Sensors::update_data_temperature(float &temperature_data){

    // gets temperature reading from temperature sensor in degrees celsius
    ds18b20_0->requestTemperatures();
    temperature_data = ds18b20_0->getTempC();

    // OneWire communication failure or DS18B20 is not connected properly
    if (temperature_data == ERROR_DISCONNECT_DS18B20) {
        temperature_data = -255;
    }
    
}

void Sensors::update_data_battery_level(float &battery_charge_data) {

    uint16_t battery_voltage_read = 0;

    // Starts I2C communication
    Wire.begin();
    Wire.beginTransmission(INA3221_I2C_ADDR);
    Wire.write(INA3221_CHANNEL_3_ADDR);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)INA3221_I2C_ADDR, (uint8_t)2);

    // read voltage 
    // if initial communication does not return a 2 then there is a problem with 
    // the I2C Communication
    if(Wire.available() != 2) {
        // I2C communication error
        sensor_class_data.battery_charge = ERROR_COMMUNICATION_I2C;
    } 
    else {
        battery_voltage_read = (Wire.read() << 8) | Wire.read();
        // Voltage to battery charge conversion. 
        // This will need to be monitored throughout the life of the battery.
        float battery_result = 95.6 * ((float)battery_voltage_read * .001) - 998;

        // If the battery reading is less than 0 or greater than 100, 
        // one or more of the following has occured in no particular order:
        // 1. INA3221 Failure
        // 2. INA3221 not calibrated to battery properly
        // 3. Incorrect conversion formula or formula needs calibration
        if ((int)battery_result < 0) {
            battery_result = ERROR_BATTERY_DISCONNECT;
        }
        
        battery_charge_data = battery_result;
    }

}