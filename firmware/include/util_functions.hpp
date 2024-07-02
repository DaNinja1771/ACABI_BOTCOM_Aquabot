#ifndef UTIL_FUNCTIONS_HPP
#define UTIL_FUNCTIONS_HPP

#include "BOTCOM_parameters.hpp"

struct debug_data{
    bool print_receive_arr;
    bool print_send_arr;
};

extern debug_data debug_var;

// constrain heading to -180 to 180
void wrap_heading_180(double &heading);

void set_mission_state_new(PlannerData &planner_data, MissionState new_state);

void debug_log(const char* format, ...);
void debug_logln(const char* format, ...);
void debug_log(int offset, const char* format, ...);
void debug_logln(int offset, const char* format, ...);
void set_log_mode(bool active);

void print_debug_dump_general(EstimatorData &estimator_data, PlannerData &planner_data, SensorData &SensorData, NetworkData &network_data);

extern const char* request_type_names[4];
extern const char* mission_state_type_names[4];
extern const char* network_state_names[6];
extern const char* send_receive_codes[11];

#endif