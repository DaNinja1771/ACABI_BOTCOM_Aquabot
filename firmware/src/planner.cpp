#include "planner.hpp"

Planner::Planner(){

}

void Planner::setup(PlannerData &planner_data){
    planner_data.state = MissionState::Initialized;
    planner_data.current_mission_request_type = RequestType::none;
    planner_data.loop_waypoints = false;
}   


void Planner::iteration(EstimatorData &estimator_data, PlannerData &planner_data, NetworkData &network_data)  {    

    if (planner_data.current_mission_request_type == RequestType::Start){
        
        if((!estimator_data.compass_data.is_updated || !estimator_data.gps_data.is_updated || !network_data.satellite_working)){
            set_mission_state_new(planner_data, MissionState::Error_Paused);
        }
        else{
            set_mission_state_new(planner_data, MissionState::Started);
        }
    }
    else if(planner_data.current_mission_request_type == RequestType::Stop){
        set_mission_state_new(planner_data, MissionState::Stopped);
    }
    else if(planner_data.current_mission_request_type == RequestType::none){
        set_mission_state_new(planner_data, MissionState::Initialized);
    }


    while (!waypointsDone(planner_data)) {
        planner_data.current_waypoint = currentGeoWaypoint(planner_data);
        
        estimator_data.next_waypoint_distance = TinyGPSPlus::distanceBetween(estimator_data.gps_data.latitude, estimator_data.gps_data.longitude,
                                                        planner_data.current_waypoint.latitude, planner_data.current_waypoint.longitude);

        if (estimator_data.next_waypoint_distance < PROXIMITY_THRESHOLD) {
           
                planner_data.geo_waypoint_index++;   
        }

        else {

            break;
        }
    }
    
    if (waypointsDone(planner_data) && planner_data.current_mission_request_type != RequestType::none) {
        // debug_logln(1, "Last waypoint reached. Mission stopped.");
        if (!planner_data.loop_waypoints){
            set_mission_state_new(planner_data, MissionState::Stopped);
        }
        else{
            planner_data.geo_waypoint_index = 0;
        }
        
    }

    estimator_data.next_waypoint_desired_heading = TinyGPSPlus::courseTo( estimator_data.gps_data.latitude, estimator_data.gps_data.longitude, 
                                                    planner_data.current_waypoint.latitude, planner_data.current_waypoint.longitude);
    
    estimator_data.compass_data.heading_error = estimator_data.next_waypoint_desired_heading - estimator_data.compass_data.heading;

    
    wrap_heading_180(estimator_data.compass_data.heading_error);

}

bool Planner::waypointsDone(PlannerData &planner_data){
    return planner_data.geo_waypoint_index >= planner_data.number_of_geo_waypoints;
}

GeoVec2 Planner::currentGeoWaypoint(PlannerData &planner_data){
    return planner_data.geo_waypoints[planner_data.geo_waypoint_index];
}
