#ifndef BOTCOM_SENSORS_HPP
#define BOTCOM_SENSORS_HPP

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "DS18B20.h"

#include "BOTCOM_parameters.hpp"

// Uncomment to run in debug mode
// #define SENSOR_DEBUG_MODE

// Uncomment if you want to use test data rather than real data
#define SENSOR_TEST_DATA

// INA3221 Addresses
#define INA3221_I2C_ADDR        0x40
#define INA3221_CHANNEL_1_ADDR  0x02
#define INA3221_CHANNEL_2_ADDR  0x06
#define INA3221_CHANNEL_3_ADDR  0x0A

// Wind Speed Sensor Parameters
#define WS_SAMPLE_PERIOD_MILLIS 500
#define WS_TRIGGER_SPEED_KPH    2.4
#define BIAS                    3.58
#define WD_RESOLUTION           22.5

// Salinity Sensor Parameters
#define SALINITY_FACTOR     .5
#define SALINITY_KVALUE     1.33

// Communication Bus Errors
#define ERROR_DISCONNECT_DS18B20    -127
#define ERROR_COMMUNICATION_ONEWIRE -256
#define ERROR_COMMUNICATION_I2C     -255
#define ERROR_BATTERY_DISCONNECT    -254

#define ERROR_PH_SENSOR         -255
#define ERROR_TDS_SENSOR        -255
#define ERROR_WIND_DIRECTION    -255
#define ERROR_WIND_SPEED        -255

// 3.3 / 4096 
// Operating voltage of 3.3 divided by 12-bit max ADC value
#define VOLTAGE_ADC_3312    0.0008056640625f

// Wind Direction Sensor angles w/ adc values upper and lower bounds
#define WD_ADC_HIGH_0_0     3119
#define WD_ADC_HIGH_22_5    1675
#define WD_ADC_HIGH_45_0    1800
#define WD_ADC_HIGH_67_5    206
#define WD_ADC_HIGH_90_0    290
#define WD_ADC_HIGH_112_5   147
#define WD_ADC_HIGH_135_0   730
#define WD_ADC_HIGH_157_5   415
#define WD_ADC_HIGH_180_0   1050
#define WD_ADC_HIGH_202_5   949 
#define WD_ADC_HIGH_225_0   2500
#define WD_ADC_HIGH_247_5   2357
#define WD_ADC_HIGH_270_0   3940
#define WD_ADC_HIGH_292_5   3450  
#define WD_ADC_HIGH_315_0   3750
#define WD_ADC_HIGH_337_5   2870

#define WD_ADC_LOW_0_0      2900
#define WD_ADC_LOW_22_5     1470      
#define WD_ADC_LOW_45_0     1676
#define WD_ADC_LOW_67_5     150   
#define WD_ADC_LOW_90_0     207
#define WD_ADC_LOW_112_5    80       
#define WD_ADC_LOW_135_0    416
#define WD_ADC_LOW_157_5    320  
#define WD_ADC_LOW_180_0    950
#define WD_ADC_LOW_202_5    750  
#define WD_ADC_LOW_225_0    2350
#define WD_ADC_LOW_247_5    2130 
#define WD_ADC_LOW_270_0    3751
#define WD_ADC_LOW_292_5    3130 
#define WD_ADC_LOW_315_0    3420
#define WD_ADC_LOW_337_5    2570

#define VOLTAGE_READING_ARRAY_SIZE 10

class Sensors{

    public:
        // Default constructor no important actions
        Sensors();
        // Destructor that deletes the memory allocated
        // for OneWire and DS18B20 class instances
        ~Sensors();
        // Handles the rest of the class set up
        void setup();
        // Updates class variable sensor_class_data to the current 
        // sensor readings
        void update_sensor_data(SensorData &sensor_data);
        // Returns the data contained in class variable sensor_class_data.

        #ifdef SENSOR_TEST_DATA
        void update_sensor_data_test(SensorData &sensor_data);
        #endif

    private:
        // Updates the pH member of sensor_class_data 
        // to the current read value from the pH sensor
        void update_data_ph(float &ph_data);  
        // Updates the salinity member of sensor_class_data 
        // to the current read value from the salinity sensor
        void update_data_salinity(float &salinity_data);
        // Updates the temperature member of sensor_class_data 
        // to the current read value from the temperature sensor
        void update_data_temperature(float &temperature_data);
        // Updates the wind_speed member of sensor_class_data 
        // to the current read value from the wind speed sensor
        void update_data_wind_speed(float &wind_speed_data);
        // Updates the wind_direction member of sensor_class_data 
        // to the current read value from the wind direction sensor
        // and takes an input of the heading to get corrected direction
        void update_data_wind_direction(float &wind_direction_data);
        // Updates the battery_charge member of sensor_class_data 
        // to the current read value from the battery charge sensor
        void update_data_battery_level(float &battery_charge_data);
        
        // gets an average of 10 analog readings with a 
        // 10 milisecond seperation between analod reads
        uint16_t average_value_analog(uint16_t pin_value);
        // Allows for the wind speed sensor to trigger an interrupt.
        // Must be a static function.
        static void wind_speed_interrupt();
        // Sets the number of rotations made by the wind speed sensor
        // within a window period defined by WS_SAMPLE_PERIOD_MILLIS. 
        void update_wind_speed();
        
        // Contains all the sensor data once the update_sensor_data(float &heading)
        // function is called.
        SensorData sensor_class_data;
        // Start time of current window in milliseconds
        uint32_t wind_speed_previous_time;
        // Number of rotations of wind speed sensor in new measured window
        uint32_t wind_speed_count;
        // Number of rotations of wind speed sensor in current measured window
        uint32_t wind_speed_count_prev;
        // Class instances of OneWire and DS18B20
        OneWire *one_wire;
        DS18B20 *ds18b20_0;
        // Allows temperature sensor to function properly
        static Sensors* ds18b20_instance;
        // Upper bounds of values for ADC values found with the wind direction sensor
        uint16_t wind_direction_adc_high[16];

        // Lower bounds of values for ADC values found with the wind direction sensor
        uint16_t wind_direction_adc_low[16];
        
        unsigned long millis_start = 0;
        unsigned int battery_subber = 0;
}; // End Sensors Class

#endif // End BOTCOM_PARAMETERS_HPP