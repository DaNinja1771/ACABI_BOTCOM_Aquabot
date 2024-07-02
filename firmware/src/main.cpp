#include "BOTCOM_main.hpp"

Estimator estimator;
Sensors sensors;
Planner planner;
Controller controller;
Network network;

SensorData g_sensor_data;
EstimatorData g_estimator_data;
PlannerData g_planner_data;
NetworkData g_network_data;

StaticSemaphore_t network_data_mutex_buffer;
SemaphoreHandle_t network_data_mutex = NULL;

StaticSemaphore_t processed_request_mutex_buffer;
SemaphoreHandle_t processed_request_mutex = NULL;

StaticSemaphore_t satellite_error_mutex_buffer;
SemaphoreHandle_t satellite_error_mutex = NULL;

TaskHandle_t sat_task_handle = NULL;

void SatCom_iteration(void *parameters){
    while(1){

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xSemaphoreTake(network_data_mutex, portMAX_DELAY) == pdTRUE){
            network.iteration_3();
            xSemaphoreGive(network_data_mutex);
        }
    }
}

void setup() {

    Serial.begin(115200);
    delay(5000);

    sensors.setup();

    estimator.setup(g_estimator_data);
    planner.setup(g_planner_data);
    
    controller.setup();
    network.setup();

    estimator.compass_calibration_auto(controller, g_estimator_data);

    network_data_mutex = xSemaphoreCreateMutexStatic(&network_data_mutex_buffer);
    processed_request_mutex = xSemaphoreCreateMutexStatic(&processed_request_mutex_buffer);
    satellite_error_mutex = xSemaphoreCreateMutexStatic(&satellite_error_mutex_buffer);

    // default core is 0
    xTaskCreatePinnedToCore(
        SatCom_iteration,      // task name
        "sat_com",          // task name id
        4096,               // Stack size, can be smaller if needed
        NULL,               // Task parameters
        1,                  // Priority
        &sat_task_handle,   // Task handle
        1                   // Core
    );

    // since we are using arduino.h we dont need to call this.
    // if we remove arduino.h then we need to call this
    // vTaskStartScheduler();

}// End Setup Function

void loop() {

    static unsigned long lastMainLoopMillis = 0;
    static unsigned long lastSubLoopMillis = 0;

    if (millis() - lastMainLoopMillis >= MAIN_LOOP_PERIOD){
        lastMainLoopMillis = millis();
        
        estimator.gps_iteration(g_estimator_data.gps_data);
        estimator.compass_iteration(g_estimator_data.compass_data);

        planner.iteration(g_estimator_data, g_planner_data, g_network_data);
        controller.iteration(g_planner_data, g_estimator_data);

        sensors.update_sensor_data(g_sensor_data);
        
        if (xSemaphoreTake(network_data_mutex, 10) == pdTRUE){
            if (sat_task_handle != NULL){
                network.sync_network_data(g_estimator_data, g_planner_data, g_sensor_data, g_network_data);
                xSemaphoreGive(network_data_mutex);
                xTaskNotifyGive(sat_task_handle);
                delay(5);
            }
        }
        else{
            g_network_data.network_state = network.get_network_state();
        }
        print_debug_dump_general(g_estimator_data, g_planner_data, g_sensor_data, g_network_data);
    }// End of Main Loop

    if (millis() - lastSubLoopMillis >= MINOR_LOOP_NAV_PERIOD){
        lastSubLoopMillis = millis();
        set_log_mode(false);

        estimator.compass_iteration(g_estimator_data.compass_data);
        controller.iteration(g_planner_data, g_estimator_data);

        set_log_mode(true);
    }// End of Minor Loop

}// End Loop Function