#include "BOTCOM_sensors.hpp"

Sensors* Sensors::ds18b20_instance = nullptr;

Sensors::Sensors(){

}

Sensors::~Sensors(){

    delete ds18b20_0;
    delete one_wire;
}

void Sensors::setup(){

    pinMode(SENSORS_PIN_PH, INPUT);
    pinMode(SENSORS_PIN_SALINITY, INPUT);
    pinMode(SENSORS_PIN_WIND_DIRECTION, INPUT);
    pinMode(SENSORS_PIN_WIND_SPEED, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(SENSORS_PIN_WIND_SPEED), wind_speed_interrupt, CHANGE);

    one_wire = new OneWire(ONEWIRE_PIN_BUS);

    ds18b20_0 = new DS18B20(one_wire);
    ds18b20_0->setResolution(12);
    ds18b20_instance = this;
    ds18b20_0->begin();
    
    wind_direction_adc_high[0] = WD_ADC_HIGH_0_0;       // 0
    wind_direction_adc_high[1] = WD_ADC_HIGH_22_5;      // 22.5
    wind_direction_adc_high[2] = WD_ADC_HIGH_45_0;      // 45
    wind_direction_adc_high[3] = WD_ADC_HIGH_67_5;      // 67.5
    wind_direction_adc_high[4] = WD_ADC_HIGH_90_0;      // 90
    wind_direction_adc_high[5] = WD_ADC_HIGH_112_5;     // 112.5
    wind_direction_adc_high[6] = WD_ADC_HIGH_135_0;     // 135
    wind_direction_adc_high[7] = WD_ADC_HIGH_157_5;     // 157.5
    wind_direction_adc_high[8] = WD_ADC_HIGH_180_0;     // 180
    wind_direction_adc_high[9] = WD_ADC_HIGH_202_5;     // 202.5
    wind_direction_adc_high[10] = WD_ADC_HIGH_225_0;    // 225
    wind_direction_adc_high[11] = WD_ADC_HIGH_247_5;    // 247.5
    wind_direction_adc_high[12] = WD_ADC_HIGH_270_0;    // 270
    wind_direction_adc_high[13] = WD_ADC_HIGH_292_5;    // 292.5
    wind_direction_adc_high[14] = WD_ADC_HIGH_315_0;    // 315
    wind_direction_adc_high[15] = WD_ADC_HIGH_337_5;    // 337.5

    wind_direction_adc_low[0] = WD_ADC_LOW_0_0;         // 0
    wind_direction_adc_low[1] = WD_ADC_LOW_22_5;        // 22.5
    wind_direction_adc_low[2] = WD_ADC_LOW_45_0;        // 45
    wind_direction_adc_low[3] = WD_ADC_LOW_67_5;        // 67.5
    wind_direction_adc_low[4] = WD_ADC_LOW_90_0;        // 90
    wind_direction_adc_low[5] = WD_ADC_LOW_112_5;       // 112.5
    wind_direction_adc_low[6] = WD_ADC_LOW_135_0;       // 135
    wind_direction_adc_low[7] = WD_ADC_LOW_157_5;       // 157.5
    wind_direction_adc_low[8] = WD_ADC_LOW_180_0;       // 180
    wind_direction_adc_low[9] = WD_ADC_LOW_202_5;       // 202.5
    wind_direction_adc_low[10] = WD_ADC_LOW_225_0;      // 225
    wind_direction_adc_low[11] = WD_ADC_LOW_247_5;      // 247.5
    wind_direction_adc_low[12] = WD_ADC_LOW_270_0;      // 270
    wind_direction_adc_low[13] = WD_ADC_LOW_292_5;      // 292.5
    wind_direction_adc_low[14] = WD_ADC_LOW_315_0;      // 315
    wind_direction_adc_low[15] = WD_ADC_LOW_337_5;      // 337.5

    wind_speed_previous_time = millis(); 
}

void Sensors::update_sensor_data(SensorData &sensor_data){

    update_data_temperature(sensor_data.temperature);
    update_data_ph(sensor_data.pH);
    update_data_salinity(sensor_data.salinity);
    update_data_wind_speed(sensor_data.windSpeed);
    update_data_wind_direction(sensor_data.windDirection);
    update_data_battery_level(sensor_data.battery_charge);
}

uint16_t Sensors::average_value_analog(uint16_t pin_value){

    uint16_t average_voltage = 0;
    uint16_t voltage_reading_arr[VOLTAGE_READING_ARRAY_SIZE];
    uint16_t temporary = 0;
    // gets 10 analog readings from sensor and stores them into an array
    for(uint8_t i = 0; i < VOLTAGE_READING_ARRAY_SIZE; i++){   

        voltage_reading_arr[i] = analogRead(pin_value);
        delay(2);
    }
    // sorts the analog value array
    for(uint8_t i = 0; i < VOLTAGE_READING_ARRAY_SIZE-1; i++){

        for(uint8_t j = 0; j < VOLTAGE_READING_ARRAY_SIZE; j++){

            if(voltage_reading_arr[i] > voltage_reading_arr[j]){

                temporary = voltage_reading_arr[i];
                voltage_reading_arr[i] = voltage_reading_arr[j];
                voltage_reading_arr[j] = temporary;
            }
        }
    }
    // truncate the sorted analog value array to remove potential error readings
    for(uint8_t i = 2; i < 8; i++){

        average_voltage += voltage_reading_arr[i];
    }
    // find the average analog value
    average_voltage = average_voltage / 6;

    return (uint16_t)average_voltage;
}

#ifdef SENSOR_TEST_DATA
void Sensors::update_sensor_data_test(SensorData &sensor_data){

    int temp = random(0, 2000);
    
    sensor_data.battery_charge    = 69.69;
    sensor_data.pH                = 42.24;
    sensor_data.salinity          = 98.1;
    sensor_data.windSpeed         = 17.56;
    sensor_data.windDirection     = 92.4;
    sensor_data.temperature       = 32.4;
    
    if(temp % 5 == 0){
        sensor_data.pH = ERROR_PH_SENSOR;
    }
    if(temp % 2 == 0){
        sensor_data.salinity = ERROR_TDS_SENSOR;
    }
    if (temp % 7 == 0){
        sensor_data.windDirection = ERROR_WIND_DIRECTION;
    }
    if(temp % 9 == 0){
        sensor_data.temperature = ERROR_DISCONNECT_DS18B20;
    }
    if(temp % 3 == 0){
        sensor_data.windSpeed = ERROR_WIND_SPEED;
    }
    if(temp % 6 == 0){
        sensor_data.battery_charge = ERROR_BATTERY_DISCONNECT;
    }

    delay(100);
}


#endif
