#include "arc_msgs/ToggleSchema.h"
#include "TaskGuidedCleanDebrisServer.h"
#include "dynamic_reconfigure/Config.h"
#include "boost/algorithm/string.hpp"
#include "TaskServer.h"


#define MAX_QUEUE_SIZE 1000

TaskGuidedCleanDebrisServer::TaskGuidedCleanDebrisServer() : server(global_handle, "task_guided_clean_debris", boost::bind(&TaskServer::goal_cb, this, _1), false)
{
    ros::NodeHandle local_handle("task_guided_clean_debris_server");
    ros::Timer timer = global_handle.createTimer(ros::Duration(60), &TaskGuidedCleanDebrisServer::explore_timer_cb, this, false);
    this->debris_sub = global_handle.subscribe("detect_debris_ps/debris_locations", MAX_QUEUE_SIZE, &TaskGuidedCleanDebrisServer::debris_locations_cb, this);
    this->base_pos_sub = global_handle.subscribe("base_pose_ground_truth", MAX_QUEUE_SIZE, &TaskGuidedCleanDebrisServer::base_pose_cb, this);
    this->pose_listener;

    this->debris_success_count = 0;

    this->local_handle = local_handle;
    this->abandon_failed_debris_timer = global_handle.createTimer(ros::Duration(60), &TaskGuidedCleanDebrisServer::abandon_failed_debris_timer_cb, this, false);
    this->clean_debris_timer = global_handle.createTimer(ros::Duration(60), &TaskGuidedCleanDebrisServer::clean_debris_timer_cb, this, false);
    this->clean_debris_timer.stop();
    this->abandon_failed_debris_timer.stop();
    //TODO: Handle pre-empt callback as well

    this->arc_base_client = global_handle.serviceClient<arc_msgs::ToggleSchema>("arc_base/toggle_schema");
    this->move_to_debris_client = global_handle.serviceClient<arc_msgs::NavigationRequest>("move_to_goal_ms/move_to_goal");
    this->abort_all_goals_client = global_handle.serviceClient<std_srvs::Trigger>("navigation_adapter/abort_goals");

    //Enable the move_to_goal schema since we will be using it throughout task.
    dynamic_reconfigure::BoolParameter move_to_goal_ms;
    arc_msgs::ToggleSchema request;
    move_to_goal_ms.name = "move_to_goal_ms";
    move_to_goal_ms.value = true;
    request.request.schema.push_back(move_to_goal_ms); //allow robot to wander around randomly
    this->arc_base_client.call(request);
    this->server.start();

    this->instance_state.currently_cleaning = false;
}

void TaskGuidedCleanDebrisServer::clean_debris_timer_cb(const ros::TimerEvent &event) {
    ROS_INFO("Clean debris timer cb called. Started at %d and ended at %d",event.last_real, event.current_real);
    int target_id = this->target_debris.debris_id; //the debris we tried to clean
    ROS_INFO("SEARCHING FOR DEBRIS ID %d", target_id);
    bool found = false; //Whether or not we still found the debris around us.

    for(int pos = 0; pos < this->debris_found_nearby.debris.size(); pos++) {
        ROS_INFO("cycled through debris with id %d", this->debris_found_nearby.debris.at(pos).debris_id);
        if(this->debris_found_nearby.debris.at(pos).debris_id==target_id) {
            found = true; //uh oh... it's not clean then...
        }
    }

    if(found) { //If we found the debris, then that means we failed to clean it
        this->state = STATE_FailedDebrisRemoval;
    } else {
        this->state = STATE_DoneDebrisRemoval;
    }
    this->instance_state.currently_cleaning = false;
    this->clean_debris_timer.stop();
}

void TaskGuidedCleanDebrisServer::base_pose_cb(const nav_msgs::Odometry &odom) {
    this->recent_pose = odom;
}

void TaskGuidedCleanDebrisServer::explore_timer_cb(const ros::TimerEvent &event) {
    //State transition
    ROS_INFO("Explore timer cb called. Started at %d and ended at %d",event.last_real, event.current_real);
    this->shutdown();
}

void TaskGuidedCleanDebrisServer::shutdown() {
    arc_msgs::ToggleSchema request;

    //disable schemas
    dynamic_reconfigure::BoolParameter schema_clean;
    schema_clean.name = "clean_debris_ms";
    schema_clean.value = false;
    request.request.schema.push_back(schema_clean);

    this->arc_base_client.call(request);

    //turn off flags
    this->result.task_id = this->recent_goal.task_id;
    this->instance_state.currently_cleaning = false;
    this->instance_state.currently_seeking_debris = false;
    this->instance_state.currently_abandoning = false;

    this->server.setSucceeded(this->result);
}

void TaskGuidedCleanDebrisServer::debris_locations_cb(const arc_msgs::DetectedDebris &debris) {
    this->debris_found_nearby = debris;
}

void TaskGuidedCleanDebrisServer::abandon_failed_debris_timer_cb(const ros::TimerEvent &event) {
    this->abandon_failed_debris_timer.stop();
    this->instance_state.currently_abandoning = false;
    this->state = STATE_SelectDebrisTarget;
}

std::string TaskGuidedCleanDebrisServer::stateToString(TaskGuidedCleanDebrisServer::State state) {
    std::string result = "";
    if (state == STATE_SelectDebrisTarget) {
        result = "SelectDebrisTarget";
    } else if (state == STATE_SeekingDebrisLocation) {
        result = "SeekingDebrisLocation";
    } else if (state == STATE_RemovingDebris) {
        result = "RemovingDebris";
    } else if (state == STATE_DoneDebrisRemoval) {
        result = "DoneDebrisRemoval";
    } else if (state == STATE_FailedDebrisRemoval) {
        result = "FailedDebrisRemoval";
    } else if (state == STATE_AbandonFailedDebris) {
        result = "AbandonFailedDebris";
    }
    return result;
}

void TaskGuidedCleanDebrisServer::StateSelectDebrisTarget() {
    if(this->debris_list.debris.empty()) {
        arc_msgs::ArcTaskActionResult result;

        if(this->debris_count - this->debris_success_count == 0) { //no failed attempts
            result.result.completed = true;
        } else {
            result.result.completed = false;
        }

        this->result.final_state = stateToString(this->state).c_str();
        this->server.setSucceeded(this->result);

        this->shutdown();
    } else {
        //Find the debris closest to us
        this->target_debris =this->debris_list.debris.at(0);
        this->target_debris.debris_id = this->debris_list.debris.at(0).debris_id;
        int nearest_debris_pos = 0; //position of the nearest debris

        for(int pos = 1; pos < this->debris_list.debris.size(); pos++) {
            arc_msgs::Debris curr = this->debris_list.debris.at(pos);
            double curr_distance_away = sqrt(pow(curr.pose.position.x,2) + pow(curr.pose.position.y,2));
            double min_distance_away = sqrt(pow(this->target_debris.pose.position.x, 2) + pow(this->target_debris.pose.position.y,2));
            ROS_DEBUG("checking debris at location %f, %f", curr.pose.position.x, curr.pose.position.y);

            //compare the distance of current debris, and the current minimum to see if we have a new min
            if(curr_distance_away < min_distance_away) {
                this->target_debris = curr;
                this->target_debris.debris_id = curr.debris_id;
                nearest_debris_pos = pos;
            }
        }
        this->target_pose.position = this->target_debris.pose.position;
        this->target_pose.orientation.w = 1.0;

        //remove the nearest debris from the list now so we don't look for it later.
        this->debris_list.debris.erase(debris_list.debris.begin()+nearest_debris_pos);

        //Now we head to where this debris is. This is giving us info about debris in position relative (in front of us)
        ROS_INFO("Planning on going to location (%f, %f) to find debris with id %d", target_pose.position.x, target_pose.position.y, this->target_debris.debris_id);
        this->state = STATE_SeekingDebrisLocation;
    }
}

void TaskGuidedCleanDebrisServer::startup(const arc_msgs::ArcTaskGoalConstPtr &goal) {
    const char REQUEST_DELIMITER = '|';
    ROS_INFO("Starting up task_guidedcleandebris.");

    try {
        //handling string parameters
        ROS_INFO("Trying to cycle through %d debris items", (int)goal->parameters.ints.size());
        for (int pos = 0; pos < goal->parameters.strs.size(); pos++) {
            dynamic_reconfigure::StrParameter curr_param = goal->parameters.strs.at(pos);

            if (curr_param.name == "debris_list") {
                int pos = 0;
                std::string curr_coordinate;
                ROS_INFO("Checking debris parameter list");

                this->debris_list.debris = parseDebrisList(curr_param.value).debris;
            }
        }
    } catch(std::invalid_argument &e) {
        ROS_WARN("%s",e.what());
    }

    this->state = STATE_SelectDebrisTarget;
}

arc_msgs::DetectedDebris TaskGuidedCleanDebrisServer::parseDebrisList(std::string input) {
//Go through each coordinate pair "(x,y)"
    arc_msgs::DetectedDebris debris_list;
    int pos = 0;
    std::string curr_coordinate;

    while ((pos = input.find("|")) != std::string::npos) {
        curr_coordinate = input.substr(0, pos);
        curr_coordinate.erase(0, 1); //get rid of leading (
        curr_coordinate.erase(curr_coordinate.size() - 1, curr_coordinate.size()); //get rid of rightmost )

        int first_comma_pos = curr_coordinate.find(",");
        int second_comma_pos = curr_coordinate.find(",", first_comma_pos + 1);

        //extracting the actual coordinates. finally... geez c++, verbose much?
        int id = atoi(curr_coordinate.substr(0,first_comma_pos).c_str());
        double x = atof(curr_coordinate.substr(first_comma_pos + 1, second_comma_pos).c_str());
        double y = atof(curr_coordinate.substr(second_comma_pos + 1, curr_coordinate.size()).c_str());

        //store this debris in list so we know to clean it soon
        arc_msgs::Debris curr_debris;
        curr_debris.debris_id = id;
        curr_debris.pose.position.x = x;
        curr_debris.pose.position.y = y;
        curr_debris.pose.orientation.w = 1.0;

        ROS_INFO("read debris parameter (debris_id=%d, x_pos=%f, y_pos=%f)",id, x,y);
        debris_list.debris.push_back(curr_debris);

        ROS_ASSERT(x >= 0 && y >= 0);

        input.erase(0, pos + 1); //+1 because request delimiter is length 1 char.
    }

    ROS_INFO("Left with this for string %s", input.c_str());

    //assume just a single parameter here
    if(input.size()>0) {
        int first_comma_pos = input.find(",");
        int second_comma_pos = input.find(",", first_comma_pos + 1);

        //extracting the actual coordinates. finally... geez c++, verbose much?
        int id = atoi(input.substr(1,first_comma_pos).c_str());
        double x = atof(input.substr(first_comma_pos + 1, second_comma_pos).c_str());
        double y = atof(input.substr(second_comma_pos + 1, input.size()).c_str());

        //store this debris in list so we know to clean it soon
        arc_msgs::Debris curr_debris;
        curr_debris.debris_id = id;
        curr_debris.pose.position.x = x;
        curr_debris.pose.position.y = y;
        curr_debris.pose.orientation.w = 1.0;

        ROS_INFO("Requested Guided_Clean_Debris task with parameter (debris_id=%d, x_pos=%f, y_pos=%f)",id, x,y);
        debris_list.debris.push_back(curr_debris);

        ROS_ASSERT(x >= 0 && y >= 0);
    }

    for(int pos=0; pos < debris_list.debris.size(); pos++) {
        ROS_INFO("Parsed the debris at (%d, %d)", (int)debris_list.debris.at(pos).pose.position.x, (int)debris_list.debris.at(pos).pose.position.y);
    }

    return debris_list;
}

bool TaskGuidedCleanDebrisServer::decodeStringParameter(std::string name, std::string value) {
    ROS_INFO("Parameter named %s with value %s", name.c_str(), value.c_str());
}

void TaskGuidedCleanDebrisServer::process() {
    ros::Rate r(10);

    //toggling of server (setSucceeded() etc, should only be done in this loop, not within state methods)
    while(ros::ok() && server.isActive()) {
        ros::spinOnce();
        if (state == STATE_SelectDebrisTarget) {
            ROS_INFO_ONCE("IN STATE: SelectDebrisTarget");
            StateSelectDebrisTarget();
        } else if (state == STATE_SeekingDebrisLocation) {
            StateSeekingDebrisLocation();
        } else if (state == STATE_RemovingDebris) {
            ROS_INFO_ONCE("IN STATE: RemovingDebris.");
            StateRemovingDebris();
        } else if (state == STATE_DoneDebrisRemoval) {
            ROS_INFO_ONCE("IN STATE: DoneDebrisRemoval.");
            StateDoneDebrisRemoval();
        } else if (state == STATE_FailedDebrisRemoval) {
            ROS_INFO_ONCE("IN STATE: FailedDebrisRemoval.");
            StateFailedDebrisRemoval();
        } else if (state == STATE_AbandonFailedDebris) {
            ROS_INFO_ONCE("IN STATE: AbandonFailedDebris.");
            StateAbandonFailedDebris();
        }
        r.sleep();
    }
}

void TaskGuidedCleanDebrisServer::StateSeekingDebrisLocation() {
    //only send a goal if we have not sent one already
    if(!this->instance_state.currently_seeking_debris) {
        //Enable the move_to_goal schema so we can move to the debris
        dynamic_reconfigure::BoolParameter move_to_goal_ms;
        arc_msgs::ToggleSchema request;
        move_to_goal_ms.name = "move_to_goal_ms";
        move_to_goal_ms.value = true;
        request.request.schema.push_back(move_to_goal_ms); //allow robot to wander around randomly
        this->arc_base_client.call(request);

        this->instance_state.currently_seeking_debris = true;
        arc_msgs::NavigationRequest req;
        req.request.pose.position.x = this->target_pose.position.x;
        req.request.pose.position.y = this->target_pose.position.y;

        ROS_INFO("Sending navigation request to location (%f, %f)", req.request.pose.position.x, req.request.pose.position.y);
        this->move_to_debris_client.call(req);

    } else { //we are heading to the debris
        //check our current distance from debris
        double curr_x = this->recent_pose.pose.pose.position.x;
        double curr_y = this->recent_pose.pose.pose.position.y;
        double distance_from_debris = sqrt(pow(curr_x - target_pose.position.x, 2) + pow(curr_y - target_pose.position.y,2));
        ROS_INFO("Currently %f meters away from the debris.", distance_from_debris);

        if(distance_from_debris < this->stopping_distance_from_debris) {
            ROS_INFO("CLOSE ENOUGH TO DEBRIS TO STOP!");
            std_srvs::Trigger abort_trigger;
            this->abort_all_goals_client.call(abort_trigger);

            //now we can start cleaning
            this->state = STATE_RemovingDebris;
            this->instance_state.currently_seeking_debris = false;
        }
    }

    //check to see once we are close enough to the debris location
    ROS_INFO_ONCE("Planning on going to position (%f, %f)", this->target_pose.position.x, this->target_pose.position.y);
}

void TaskGuidedCleanDebrisServer::StateRemovingDebris() {
    //check to see if we found our target debris nearby. We need to see it at some point to ensure it's actually there to clean.
    for(int pos=0; pos < this->debris_found_nearby.debris.size(); pos++) {
        if(debris_found_nearby.debris.at(pos).debris_id==this->target_debris.debris_id) {
            this->instance_state.found_debris_target = true;
        }
    }
    if(!this->instance_state.currently_cleaning) {
        if(!this->instance_state.found_debris_target) {
            ROS_INFO("Moved to location (%f, %f) but could not find debris with id %d. Moving on.", (float)target_debris.pose.position.x, (float)target_debris.pose.position.y, (int)target_debris.debris_id);
            this->state = STATE_SelectDebrisTarget;
            this->instance_state.found_debris_target = false;
        } else {
            dynamic_reconfigure::BoolParameter clean_debris_ms;
            arc_msgs::ToggleSchema request;
            clean_debris_ms.name = "clean_debris_ms";
            clean_debris_ms.value = true;
            request.request.schema.push_back(clean_debris_ms); //allow robot to wander around randomly
            this->arc_base_client.call(request);
            //we call the timer, once it's done the callback will be called, and next state is triggered.
            this->clean_debris_timer.setPeriod(ros::Duration(this->cleaning_debris_time));
            this->clean_debris_timer.start();
            this->instance_state.currently_cleaning = true;
        }
    }
}

void TaskGuidedCleanDebrisServer::StateDoneDebrisRemoval() {
    this->debris_success_count++;
    this->debris_found_nearby.debris.erase(this->debris_found_nearby.debris.begin(), this->debris_found_nearby.debris.end());
    ROS_INFO("Succesfully cleaned debris. Total cleaned = %d", this->debris_success_count);
    this->target_debris = {}; //reset to defaults.

    //disable debris cleaner
    dynamic_reconfigure::BoolParameter clean_debris_ms;
    arc_msgs::ToggleSchema request;
    clean_debris_ms.name = "clean_debris_ms";
    clean_debris_ms.value = false;
    request.request.schema.push_back(clean_debris_ms); //allow robot to wander around randomly
    this->arc_base_client.call(request);

    this->instance_state.currently_cleaning = false;

    ROS_ASSERT(!this->instance_state.currently_cleaning);
    this->state = STATE_SelectDebrisTarget;
}

void TaskGuidedCleanDebrisServer::StateFailedDebrisRemoval() {
    ROS_INFO("Failed to clear debris.");
    dynamic_reconfigure::BoolParameter clean_debris_ms;
    arc_msgs::ToggleSchema request;
    clean_debris_ms.name = "clean_debris_ms";
    clean_debris_ms.value = false;
    request.request.schema.push_back(clean_debris_ms); //allow robot to wander around randomly
    this->arc_base_client.call(request);

    this->state = STATE_AbandonFailedDebris;
}

void TaskGuidedCleanDebrisServer::StateAbandonFailedDebris() {
    //enable random wandering for some amount of time so we can escape debris
    const double ABANDON_DURATION = 5.0; //explore for 5 seconds when we can't clean a debris

    this->target_debris = {};
    if(!this->instance_state.currently_abandoning) {
        dynamic_reconfigure::BoolParameter random_wander_ms;
        arc_msgs::ToggleSchema request;
        random_wander_ms.name = "random_wander_ms";
        random_wander_ms.value = true;
        request.request.schema.push_back(random_wander_ms); //allow robot to wander around randomly
        this->arc_base_client.call(request);
        this->abandon_failed_debris_timer.setPeriod(ros::Duration(ABANDON_DURATION));
        this->abandon_failed_debris_timer.start();
        this->instance_state.currently_abandoning = true;
        this->instance_state.currently_cleaning = false;
    }
}
