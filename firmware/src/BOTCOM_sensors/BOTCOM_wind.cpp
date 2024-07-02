#include "BOTCOM_sensors.hpp"

void Sensors::wind_speed_interrupt(){

    if (ds18b20_instance){
        ds18b20_instance->update_wind_speed();
        ds18b20_instance->wind_speed_count++;
    }
    
}

void Sensors::update_data_wind_speed(float &wind_speed_data){
    // Make sure to update with current window data
    update_wind_speed();

    // Converts number of rotations found in window in ms to seconds 
    // and find the speed based on 2.4 KPH constant
    float wind_speed_return = (float) wind_speed_count_prev / WS_SAMPLE_PERIOD_MILLIS;
    wind_speed_return *= 1000 * (float)WS_TRIGGER_SPEED_KPH / 2;

    if(wind_speed_return > 80 || wind_speed_return < -BIAS){
        wind_speed_data = ERROR_WIND_SPEED;
        return;
    }

    wind_speed_data = (wind_speed_return / 1.609) - BIAS;

    if (wind_speed_data <= 0){
        wind_speed_data = 0;
    }
    
}

void Sensors::update_wind_speed(){

    uint32_t current_time = millis();

    uint32_t delta_time = current_time - wind_speed_previous_time;

    if (delta_time >= (WS_SAMPLE_PERIOD_MILLIS)){

        if (delta_time > (WS_SAMPLE_PERIOD_MILLIS * 2)){
            wind_speed_previous_time = current_time;
            wind_speed_count = 0;
            wind_speed_count_prev = 0;
        }
        else {
            wind_speed_previous_time += WS_SAMPLE_PERIOD_MILLIS;
            wind_speed_count_prev = wind_speed_count;
            wind_speed_count = 0;
        }

    }
    
}

void Sensors::update_data_wind_direction(float &wind_direction_data){

    uint16_t adc_avg = 0;

    adc_avg = average_value_analog(SENSORS_PIN_WIND_DIRECTION);
    // Serial.println(adc_avg);

     for (uint8_t i = 0; i < 16; i++){
        if ((wind_direction_adc_low[i] <= adc_avg) && (adc_avg <= wind_direction_adc_high[i])){
            
            wind_direction_data = 22.5 * i;
            // wind_direction_data = fmod((wind_direction_data + heading), 360);
            // wind_direction_data = round(wind_direction_data / WD_RESOLUTION) * WD_RESOLUTION; 
            break;
        }
        else{
            wind_direction_data = ERROR_WIND_DIRECTION;
        }
    }
}