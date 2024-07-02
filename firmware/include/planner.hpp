#ifndef PLANNER_H
#define PLANNER_H

#include "BOTCOM_parameters.hpp"
#include "estimator.hpp"
#include "util_functions.hpp"

class Planner {

public:
    Planner();
    void setup(PlannerData &planner_data);
    void iteration(EstimatorData &estimator_data, PlannerData &planner_data, NetworkData &network_data);
    
private:
    
    const char* mission_state_type_names[4] = 
    {
    "Initialized", 
    "Error_Paused", 
    "Started", 
    "Stopped"
    };

    bool waypointsDone(PlannerData &planner_data);

    GeoVec2 currentGeoWaypoint(PlannerData &planner_data);

};// End Planner Class

#endif // PLANNER_H
