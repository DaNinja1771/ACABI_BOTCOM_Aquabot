#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Multi-Core Stuff
// This is set to 1000, so this is treated as milliseconds
// configTICK_RATE_HZ;

extern StaticSemaphore_t network_data_mutex_buffer;
extern SemaphoreHandle_t network_data_mutex;

extern StaticSemaphore_t processed_request_mutex_buffer;
extern SemaphoreHandle_t processed_request_mutex;

extern StaticSemaphore_t satellite_error_mutex_buffer;
extern SemaphoreHandle_t satellite_error_mutex;

extern TaskHandle_t sat_task_handle;