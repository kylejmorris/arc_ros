
#include "arc_msgs/ToggleSchema.h"
#include <arc_msgs/WirelessAnnouncement.h>
#include "TaskConfirmVictimServer.h"
#include "dynamic_reconfigure/Config.h"
#include <arc_msgs/DetectedVictims.h>
#include "boost/algorithm/string.hpp"
#include "TaskServer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#define MAX_QUEUE_SIZE 1000

TaskConfirmVictimServer::TaskConfirmVictimServer(const std::string &robotName, const std::string &customNamespace) : server(ros::NodeHandle(customNamespace), "task_confirm_victim", boost::bind(&TaskServer::goal_cb, this, _1), false)
{
    std::string topic_ns = "";
    if(robotName.size()>0) {
        topic_ns = "/arc/" + robotName+ "/";
    }

    ros::NodeHandle local_handle((topic_ns+"task_confirm_victim_server"));
    ros::NodeHandle global_handle(topic_ns);
    this->global_handle = global_handle;
    ros::Timer timer = global_handle.createTimer(ros::Duration(60), &TaskConfirmVictimServer::explore_timer_cb, this, false);
    this->victim_sub = global_handle.subscribe((topic_ns+"detect_victim_ps/found_victims"), MAX_QUEUE_SIZE, &TaskConfirmVictimServer::found_victims_cb, this);
    this->victim_status_pub = global_handle.advertise<arc_msgs::WirelessAnnouncement>((topic_ns+"wifi_handler/outgoing_announcements"), MAX_QUEUE_SIZE, true);
    this->base_pos_sub = global_handle.subscribe((topic_ns+"base_pose_ground_truth"), MAX_QUEUE_SIZE, &TaskConfirmVictimServer::base_pose_cb, this);
    this->pose_listener;

    this->victim_success_count = 0;

    this->local_handle = local_handle;
    local_handle.param<std::string>("robot_name", this->robotName, "test_bot");

    //TODO: Handle pre-empt callback as well
    this->arc_base_client = global_handle.serviceClient<arc_msgs::ToggleSchema>((topic_ns+"arc_base/toggle_schema"));
    this->move_to_debris_client = global_handle.serviceClient<arc_msgs::NavigationRequest>((topic_ns+"move_to_goal_ms/move_to_goal"));
    this->abort_all_goals_client = global_handle.serviceClient<std_srvs::Trigger>((topic_ns+"navigation_adapter/abort_goals"));

    //Enable the move_to_goal schema since we will be using it throughout task.
    dynamic_reconfigure::BoolParameter move_to_goal_ms;
    arc_msgs::ToggleSchema request;
    move_to_goal_ms.name = "move_to_goal_ms";
    move_to_goal_ms.value = true;
    request.request.schema.push_back(move_to_goal_ms); //allow robot to wander around randomly
    this->arc_base_client.call(request);
    this->server.start();

    tf2_ros::Buffer *victim_buffer = new tf2_ros::Buffer;

    tf2_ros::TransformListener *victim_listener = new tf2_ros::TransformListener(*victim_buffer);
    this->victim_listener = victim_listener;
    this->victim_buffer = victim_buffer;
}

void TaskConfirmVictimServer::base_pose_cb(const nav_msgs::Odometry &odom) {
    this->recent_pose = odom;
}

void TaskConfirmVictimServer::explore_timer_cb(const ros::TimerEvent &event) {
    //State transition
    ROS_INFO("Explore timer cb called. Started at %d and ended at %d",event.last_real, event.current_real);
    this->shutdown();
}

void TaskConfirmVictimServer::shutdown() {
    arc_msgs::ToggleSchema request;

    //disable schemas
    dynamic_reconfigure::BoolParameter schema_clean;
    schema_clean.name = "clean_debris_ms";
    schema_clean.value = false;
    request.request.schema.push_back(schema_clean);

    this->arc_base_client.call(request);

    //turn off flags
    this->result.task_id = this->recent_goal.task_id;
    this->instance_state.currently_seeking_debris = false;

    this->server.setSucceeded(this->result);
}

void TaskConfirmVictimServer::found_victims_cb(const arc_msgs::DetectedVictims &victims) {
    victims_found_nearby.victims.clear();
    //transform content
    arc_msgs::DetectedVictims victims_transformed;

    geometry_msgs::TransformStamped fiducial_link_to_map_tf;
    fiducial_link_to_map_tf = this->victim_buffer->lookupTransform("map", (this->robotName+"/base_fiducial_link"), ros::Time(0), ros::Duration(1.0));

    for(const auto &victim : victims.victims) {
        geometry_msgs::PoseStamped transformed;
        geometry_msgs::PoseStamped victim_stamped;
        victim_stamped.pose = victim.pose;
        tf2::doTransform(victim_stamped, transformed, fiducial_link_to_map_tf);

        arc_msgs::DetectedVictim transformed_victim;
        transformed_victim.victim_id = victim.victim_id;
        transformed_victim.status = victim.status;
        transformed_victim.pose = transformed.pose;

        victims_transformed.victims.push_back(transformed_victim);
    }

    for(const auto &victim  : victims_transformed.victims) {
        if(!this->victimSeenAlready(victim)) {
            ROS_INFO("found victim at %f %f", victim.pose.position.x, victim.pose.position.y);
            victims_found_nearby.victims.push_back(victim);
        }
    }
}

std::string TaskConfirmVictimServer::stateToString(TaskConfirmVictimServer::State state) {
    std::string result = "";
    if (state == STATE_SelectVictimTarget) {
        result = "SelectDebrisTarget";
    } else if (state == STATE_SeekingVictimLocation) {
        result = "SeekingDebrisLocation";
    } else if (state == STATE_DetectingVictim) {
        result = "RemovingDebris";
    }

    return result;
}

void TaskConfirmVictimServer::StateSelectVictimTarget() {
    if(this->victim_list.victims.empty()) {
        arc_msgs::ArcTaskActionResult result;

        if(this->victim_count - this->victim_success_count == 0) { //no failed attempts
            result.result.completed = true;
        } else {
            result.result.completed = false;
        }

        this->result.final_state = stateToString(this->state).c_str();
        this->server.setSucceeded(this->result);

        this->shutdown();
    } else {
        //find victim closest to us.
        int nearest_debris_pos = 0; //position of the nearest debris

        for(int pos = 0; pos < victim_list.victims.size(); pos++) {
            auto victim = victim_list.victims[pos];
            if(!this->victimSeenAlready(victim)) {
                nearest_debris_pos = pos;
                this->target_victim =this->victim_list.victims.at(pos);
                this->target_victim.status = this->victim_list.victims.at(pos).status;
            }
        }

        /** ignore this crazyness for now
        for(int pos = 1; pos < this->victim_list.victims.size(); pos++) {
            arc_msgs::DetectedVictim curr = this->victim_list.victims.at(pos);
            double curr_distance_away = sqrt(pow(curr.pose.position.x,2) + pow(curr.pose.position.y,2));
            double min_distance_away = sqrt(pow(this->target_victim.pose.position.x, 2) + pow(this->target_victim.pose.position.y,2));
            ROS_DEBUG("checking victim at location %f, %f", curr.pose.position.x, curr.pose.position.y);

            //compare the distance of current debris, and the current minimum to see if we have a new min
            if(curr_distance_away < min_distance_away) {
                this->target_victim = curr;
                this->target_victim.status = curr.status;
                nearest_debris_pos = pos;
            }
        }
         */

        this->target_pose.position = this->target_victim.pose.position;
        this->target_pose.orientation.w = 1.0;

        //remove the nearest debris from the list now so we don't look for it later. this->victim_list.debris.erase(victim_list.debris.begin()+nearest_debris_pos);

        //Now we head to where this debris is. This is giving us info about debris in position relative (in front of us)
        ROS_INFO("Planning on going to location (%f, %f) to find victim with status %d", target_pose.position.x, target_pose.position.y, this->target_victim.status);
        this->state = STATE_SeekingVictimLocation;
    }
}

void TaskConfirmVictimServer::startup(const arc_msgs::ArcTaskGoalConstPtr &goal) {
    const char REQUEST_DELIMITER = '|';
    ROS_INFO("Starting up task_guidedcleandebris.");

    //TODO: If this robot doesn't have advanced victim detector, we should exit here and reject the task
    try {
        //handling string parameters
        ROS_INFO("Trying to cycle through %d debris items", (int)goal->parameters.ints.size());
        for (int pos = 0; pos < goal->parameters.strs.size(); pos++) {
            dynamic_reconfigure::StrParameter curr_param = goal->parameters.strs.at(pos);

            if (curr_param.name == "victim_list") {
                int pos = 0;
                std::string curr_coordinate;
                ROS_INFO("Checking debris parameter list");

                this->victim_list.victims = parseVictimList(curr_param.value).victims;
            }
        }
    } catch(std::invalid_argument &e) {
        ROS_WARN("%s",e.what());
    }

    this->state = STATE_SelectVictimTarget;
}

arc_msgs::DetectedVictims TaskConfirmVictimServer::parseVictimList(std::string input) {
//Go through each coordinate pair "(x,y)"
    arc_msgs::DetectedVictims victim_list;
    int pos = 0;
    std::string curr_coordinate;

    while ((pos = input.find("|")) != std::string::npos) {
        curr_coordinate = input.substr(0, pos);
        curr_coordinate.erase(0, 1); //get rid of leading (
        curr_coordinate.erase(curr_coordinate.size() - 1, curr_coordinate.size()); //get rid of rightmost )

        int first_comma_pos = curr_coordinate.find(",");
        int second_comma_pos = curr_coordinate.find(",", first_comma_pos + 1);

        //extracting the actual coordinates. finally... geez c++, verbose much?
        int status = atoi(curr_coordinate.substr(0,first_comma_pos).c_str());
        double x = atof(curr_coordinate.substr(first_comma_pos + 1, second_comma_pos).c_str());
        double y = atof(curr_coordinate.substr(second_comma_pos + 1, curr_coordinate.size()).c_str());

        arc_msgs::DetectedVictim curr_victim;
        curr_victim.status = status;
        curr_victim.pose.position.x = x;
        curr_victim.pose.position.y = y;
        curr_victim.pose.orientation.w = 1.0;

        ROS_INFO("Requested Confirm Victim task with parameter (victim_status=%d, x_pos=%f, y_pos=%f)",status, x,y);
        victim_list.victims.push_back(curr_victim);

        ROS_ASSERT(x >= 0 && y >= 0);

        input.erase(0, pos + 1); //+1 because request delimiter is length 1 char.
    }

    ROS_INFO("Left with this for string %s", input.c_str());

    //assume just a single parameter here
    if(input.size()>0) {
        int first_comma_pos = input.find(",");
        int second_comma_pos = input.find(",", first_comma_pos + 1);

        //extracting the actual coordinates. finally... geez c++, verbose much?
        int status = atoi(input.substr(1,first_comma_pos).c_str());
        double x = atof(input.substr(first_comma_pos + 1, second_comma_pos).c_str());
        double y = atof(input.substr(second_comma_pos + 1, input.size()).c_str());

        arc_msgs::DetectedVictim curr_victim;
        curr_victim.status = status;
        curr_victim.pose.position.x = x;
        curr_victim.pose.position.y = y;
        curr_victim.pose.orientation.w = 1.0;

        ROS_INFO("Requested Confirm Victim task with parameter (victim_status=%d, x_pos=%f, y_pos=%f)",status, x,y);
        victim_list.victims.push_back(curr_victim);

        ROS_ASSERT(x >= 0 && y >= 0);
    }

    for(int pos=0; pos < victim_list.victims.size(); pos++) {
        ROS_INFO("Parsed the victim at (%d, %d)", (int)victim_list.victims.at(pos).pose.position.x, (int)victim_list.victims.at(pos).pose.position.y);
    }

    return victim_list;
}

bool TaskConfirmVictimServer::decodeStringParameter(std::string name, std::string value) {
    ROS_INFO("Parameter named %s with value %s", name.c_str(), value.c_str());
}

void TaskConfirmVictimServer::process() {
    ros::Rate r(3);

    //toggling of server (setSucceeded() etc, should only be done in this loop, not within state methods)
    while(ros::ok() && server.isActive()) {
        if (state == STATE_SelectVictimTarget) {
            ROS_INFO_ONCE("IN STATE: SelectVictimTarget");
            StateSelectVictimTarget();
        } else if (state == STATE_SeekingVictimLocation) {
            StateSeekingVictimLocation();
        } else if (state == STATE_DetectingVictim) {
            ROS_INFO_ONCE("IN STATE: DetectingVictim.");
            StateDetectingVictim();
        }


        ros::spinOnce();
        r.sleep();
    }
}

void TaskConfirmVictimServer::StateSeekingVictimLocation() {
    //only send a goal if we have not sent one already
    ROS_FATAL("SEEKING VICTIM LOCATION %f %f", this->target_pose.position.x, target_pose.position.y);

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
    } else { //we are heading to the victim
        //check our current distance from victim
        double curr_x = this->recent_pose.pose.pose.position.x;
        double curr_y = this->recent_pose.pose.pose.position.y;
        double distance_from_debris = sqrt(pow(curr_x - target_pose.position.x, 2) + pow(curr_y - target_pose.position.y,2));
        ROS_INFO("Currently %f meters away from the victim.", distance_from_debris);

        if(distance_from_debris < this->stopping_distance_from_victim) {
            ROS_INFO("CLOSE ENOUGH TO VICTIM TO STOP!");
            std_srvs::Trigger abort_trigger;
            this->abort_all_goals_client.call(abort_trigger);

            //now we can start cleaning
            this->instance_state.currently_seeking_debris = false;
            this->state = STATE_DetectingVictim;
        }
    }

    //check to see once we are close enough to the debris location
    ROS_INFO_ONCE("Planning on going to position (%f, %f)", this->target_pose.position.x, this->target_pose.position.y);
}

void TaskConfirmVictimServer::StateDetectingVictim() {
    //check if we found a victim within target range.
    arc_msgs::DetectedVictim victimFound;

    auto it = victims_found_nearby.victims.begin();
    while(it!=victims_found_nearby.victims.end()) {
        auto locationFound = it->pose.position;

        double potentialVictimDisplacement = sqrt(pow(locationFound.x-this->recent_pose.pose.pose.position.x, 2) + pow(locationFound.y-this->recent_pose.pose.pose.position.y,2));

        //TODO: should map victims coordinates to /map frame so we can calculate this displacement.
        //if we found a victim within this area we claim we found our result
        ROS_INFO("Victim displacement is %f", potentialVictimDisplacement);
        if(potentialVictimDisplacement<= MAX_VICTIM_DISPLACMENT_THRESHOLD+this->stopping_distance_from_victim) {
            this->instance_state.found_debris_target = true;
            victimFound = *it;
            it = victims_found_nearby.victims.erase(it);
            break;
        }

        it++;
    }

    //make sure to record we have now chekced this point
    ROS_FATAL("Checked %d victims", checkedVictims.victims.size());

    if(!this->instance_state.found_debris_target) {
        ROS_INFO("Moved to location (%f, %f) but could not find victim. Moving on.", (float)target_victim.pose.position.x, (float)target_victim.pose.position.y, (int)target_victim.status);
        //TODO: broadcast a failed to find victim message here.
    } else {
        this->completeConfirmingVictim(this->target_victim);
        arc_msgs::WirelessAnnouncement victimFoundAnnouncement;
        victimFoundAnnouncement.sender_location = this->recent_pose.pose.pose;

        dynamic_reconfigure::DoubleParameter xloc, yloc;
        xloc.name = "x"; xloc.value = victimFound.pose.position.x;
        yloc.name = "y"; yloc.value = victimFound.pose.position.y;
        victimFoundAnnouncement.announcement.doubles.push_back(xloc);
        victimFoundAnnouncement.announcement.doubles.push_back(yloc);

        if(victimFound.status==POSITIVE) {
            //TODO: Broadcast a success in finding victim message.. no need to clean. Just go back to finding next victim
            dynamic_reconfigure::BoolParameter positive;
            positive.name = "positive";
            positive.value = true;

            victimFoundAnnouncement.announcement.bools.push_back(positive);
            this->victim_status_pub.publish(victimFoundAnnouncement);

            ROS_INFO("The potential victim actually is a victim! CALL RESCUE TEAM.");
        } else if(victimFound.status==NEGATIVE) {
            //TODO: Broadcast a success in finding victim message.. no need to clean. Just go back to finding next victim
            ROS_FATAL("FAILED TO FIND FVICTIM");
            //while(true) {

            //}
            dynamic_reconfigure::BoolParameter positive;
            positive.name = "positive";
            positive.value = false;

            victimFoundAnnouncement.announcement.bools.push_back(positive);
            this->victim_status_pub.publish(victimFoundAnnouncement);

            ROS_INFO("The potential victim is actually not a victim... That's good.");
        } else if(victimFound.status==POTENTIAL) {
            ROS_INFO("You shouldn't have gone out looking to confirm this victim if you lack the sensor capability... silly. Also this should never happen.");
        }
    }

    this->state = STATE_SelectVictimTarget;
}

bool TaskConfirmVictimServer::alreadyCheckedVictim(const arc_msgs::DetectedVictim &victim) {
}

template<typename T>
double TaskConfirmVictimServer::dist(const T &first, const T &second) {
    return sqrt(pow(first.x - second.x, 2) + pow(first.y - second.y, 2));
}

bool TaskConfirmVictimServer::victimSeenAlready(const arc_msgs::DetectedVictim &victim) {
    bool seenBefore = false;

    //check if we already have this victim in our nearby list.
    for(const auto &currentFound : victims_found_nearby.victims) {
        if(dist(victim.pose.position, currentFound.pose.position)<MAX_VICTIM_DISPLACMENT_THRESHOLD) {
            seenBefore = true;
        }
    }

    for(const auto &other : this->checkedVictims.victims) {
        //ensure we have not seen this victim before.
        double distBetween = dist(victim.pose.position, other.pose.position);
        if (distBetween < MAX_VICTIM_DISPLACMENT_THRESHOLD) {
            seenBefore = true;
        }
    }

    return seenBefore;
}

void TaskConfirmVictimServer::completeConfirmingVictim(const arc_msgs::DetectedVictim &victim) {
    this->checkedVictims.victims.push_back(victim);

    auto victimIt = this->victim_list.victims.begin();
    while(victimIt!=this->victim_list.victims.end()) {
        if(victimSeenAlready(*victimIt)) {
            victimIt = this->victim_list.victims.erase(victimIt);
        } else {
            victimIt++;
        }
    }
}

void TaskConfirmVictimServer::announce_confirm_victim_cb(const arc_msgs::WirelessAnnouncement &msg) {
    if(msg.announcement.strs.size()>0&&msg.announcement.strs[0]
}

