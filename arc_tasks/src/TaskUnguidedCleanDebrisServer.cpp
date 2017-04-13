#include "arc_msgs/ToggleSchema.h"
#include "TaskUnguidedCleanDebrisServer.h"

#define MAX_QUEUE_SIZE 1000

TaskUnguidedCleanDebrisServer::TaskUnguidedCleanDebrisServer() : server(global_handle, "task_unguided_clean_debris", boost::bind(&TaskUnguidedCleanDebrisServer::goal_cb, this, _1), false)
{
    ros::NodeHandle local_handle("task_unguided_clean_debris");
    ros::Timer timer = global_handle.createTimer(ros::Duration(60), &TaskUnguidedCleanDebrisServer::explore_timer_cb, this, false);
    this->debris_sub = global_handle.subscribe("detect_debris_ps/debris_locations", MAX_QUEUE_SIZE, &TaskUnguidedCleanDebrisServer::debris_locations_cb, this);
    this->base_pos_sub = global_handle.subscribe("base_pose_ground_truth", MAX_QUEUE_SIZE, &TaskUnguidedCleanDebrisServer::base_pose_cb, this);
    this->pose_listener;

    this->debris_success_count = 0;

    this->local_handle = local_handle;
    this->explore_timer = timer;
    this->explore_timer.stop();
    this->abandon_failed_debris_timer = global_handle.createTimer(ros::Duration(60), &TaskUnguidedCleanDebrisServer::abandon_failed_debris_timer_cb, this, false);
    this->clean_debris_timer = global_handle.createTimer(ros::Duration(60), &TaskUnguidedCleanDebrisServer::clean_debris_timer_cb, this, false);
    this->clean_debris_timer.stop();

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

    this->currently_cleaning = false;
}

void TaskUnguidedCleanDebrisServer::clean_debris_timer_cb(const ros::TimerEvent &event) {
    ROS_INFO("Clean debris timer cb called. Started at %d and ended at %d",event.last_real, event.current_real);
    int target_id = this->target_debris.debris_id; //the debris we tried to clean
    ROS_INFO("SEARCHING FOR DEBRIS ID %d", target_id);
    bool found = false; //Whether or not we still found the debris around us.

    for(int pos = 1; pos < this->debris_list.debris.size(); pos++) {
        ROS_INFO("cycled through debris with id %d", this->debris_list.debris.at(pos).debris_id);
        if(this->debris_list.debris.at(pos).debris_id==target_id) {
            found = true; //uh oh... it's not clean then...
        }
    }

    if(found) { //If we found the debris, then that means we failed to clean it
        this->state = STATE_FailedDebrisRemoval;
    } else {
        this->state = STATE_DoneDebrisRemoval;
    }
    this->currently_cleaning = false;
    this->clean_debris_timer.stop();
}

void TaskUnguidedCleanDebrisServer::base_pose_cb(const nav_msgs::Odometry &odom) {
    this->recent_pose = odom;
}

void TaskUnguidedCleanDebrisServer::goal_cb(const arc_msgs::ArcTaskGoalConstPtr &goal) {
    ROS_INFO("Executing task_explore");

    this->startup(goal);
    this->process();
}

void TaskUnguidedCleanDebrisServer::explore_timer_cb(const ros::TimerEvent &event) {
    //State transition
    ROS_INFO("Explore timer cb called. Started at %d and ended at %d",event.last_real, event.current_real);
    this->shutdown();
}

void TaskUnguidedCleanDebrisServer::shutdown() {
    arc_msgs::ToggleSchema request;

    dynamic_reconfigure::BoolParameter schema;
    schema.name = "random_wander_ms";
    schema.value = false;
    request.request.schema.push_back(schema); //allow robot to wander around randomly

    dynamic_reconfigure::BoolParameter schema_clean;
    schema_clean.name = "clean_debris_ms";
    schema_clean.value = false;
    request.request.schema.push_back(schema_clean); //allow robot to wander around randomly

    this->arc_base_client.call(request);
    this->result.completed = true;
    this->result.final_state = stateToString(this->state).c_str();

    //TODO: This succeeded part should be set elsewhere. Whichever state the task is succeessful at
    this->server.setSucceeded(this->result);

    //turn off flags
    this->currently_cleaning = false;
    this->currently_seeking_debris = false;
    this->currently_abandoning = false;

    //ensure all timers are disabled so we don't get callbacks again.
    this->explore_timer.stop();
    this->abandon_failed_debris_timer.stop();
    this->clean_debris_timer.stop();
}

void TaskUnguidedCleanDebrisServer::abandon_failed_debris_timer_cb(const ros::TimerEvent &event) {
    this->abandon_failed_debris_timer.stop();
    this->currently_abandoning = false;
    this->state = STATE_StartExploring;
}

std::string TaskUnguidedCleanDebrisServer::stateToString(TaskUnguidedCleanDebrisServer::State state) {
    std::string result = "";
    if (state == STATE_StartExploring) {
        result = "StartExploring";
    } else if (state == STATE_Exploring) {
        result = "Exploring";
    } else if (state == STATE_FoundDebris) {
        result = "FoundDebris";
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

void TaskUnguidedCleanDebrisServer::StateStartExploring() {
    arc_msgs::ToggleSchema request;

    dynamic_reconfigure::BoolParameter random_wander_ms;
    random_wander_ms.name = "random_wander_ms";
    random_wander_ms.value = true;
    request.request.schema.push_back(random_wander_ms); //allow robot to wander around randomly
    this->arc_base_client.call(request);

    this->state = STATE_Exploring;
}

void TaskUnguidedCleanDebrisServer::startup(const arc_msgs::ArcTaskGoalConstPtr &goal) {
    this->state = STATE_StartExploring;
     ROS_INFO("setting explore time to %d", this->explore_time);
    this->explore_timer.setPeriod(ros::Duration(this->explore_time));
    this->explore_timer.start();
    ROS_INFO("Starting up task_unguidedcleandebris.");
}

void TaskUnguidedCleanDebrisServer::StateExploring() {
    //check if we found debris
    if(this->debris_list.debris.size()>0) {
        this->state = STATE_FoundDebris;
    }
}

void TaskUnguidedCleanDebrisServer::process() {
    ros::Rate r(10);

    //toggling of server (setSucceeded() etc, should only be done in this loop, not within state methods)
    while(ros::ok() && server.isActive()) {
        ros::spinOnce();
        if (state == STATE_StartExploring) {
            ROS_INFO("IN STATE: StartExploring");
            StateStartExploring();
        } else if (state == STATE_Exploring) {
            ROS_INFO_ONCE("IN STATE: Exploring");
            StateExploring();
        } else if (state == STATE_FoundDebris) {
            ROS_INFO_ONCE("IN STATE: FoundDebris");
            StateFoundDebris();
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

void TaskUnguidedCleanDebrisServer::StateFoundDebris() {
    ROS_ASSERT(this->debris_list.debris.size()>0); //we should have debris at this point.

    dynamic_reconfigure::BoolParameter random_wander_ms;
    arc_msgs::ToggleSchema request;
    random_wander_ms.name = "random_wander_ms";
    random_wander_ms.value = false;
    request.request.schema.push_back(random_wander_ms); //allow robot to wander around randomly
    this->arc_base_client.call(request);

    //Find the debris closest to us
    this->target_debris =this->debris_list.debris.at(0);

    for(int pos = 1; pos < this->debris_list.debris.size(); pos++) {
        arc_msgs::Debris curr = this->debris_list.debris.at(pos);
        double curr_distance_away = sqrt(pow(curr.pose.position.x,2) + pow(curr.pose.position.y,2));
        double min_distance_away = sqrt(pow(this->target_debris.pose.position.x, 2) + pow(this->target_debris.pose.position.y,2));
        ROS_INFO("curr distance away is %f, minimum is %f", curr_distance_away, min_distance_away);

        //compare the distance of current debris, and the current minimum to see if we have a new min
        if(curr_distance_away < min_distance_away) {
            this->target_debris = curr;
        }
    }

    //Now we head to where this debris is. This is giving us info about debris in position relative (in front of us)
    double target_x = this->target_debris.pose.position.x;
    double target_y = this->target_debris.pose.position.y;

    //Since debris messages are not stamped, we have to make a stamped one.
    geometry_msgs::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = "/test_bot/base_fiducial_link";
    stamped_pose.pose = this->target_debris.pose;


    tf::StampedTransform transform;
    geometry_msgs::PoseStamped stamped_out; //the transformed point
    try {
        pose_listener.transformPose("/test_bot/base_link",stamped_pose,stamped_out);
        stamped_out.header.frame_id = "/test_bot/base_fiducial_link";
        pose_listener.transformPose("/map",stamped_out, stamped_out);
    } catch(tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    ROS_INFO_ONCE("Found debris at (%f, %f)", stamped_out.pose.position.x, stamped_out.pose.position.y);

    this->target_pose.position.x = stamped_out.pose.position.x;
    this->target_pose.position.y = stamped_out.pose.position.y;


    this->state = STATE_SeekingDebrisLocation;
}

void TaskUnguidedCleanDebrisServer::StateSeekingDebrisLocation() {
    //only send a goal if we have not sent one already
    if(!this->currently_seeking_debris) {
        //Enable the move_to_goal schema so we can move to the debris
        dynamic_reconfigure::BoolParameter move_to_goal_ms;
        arc_msgs::ToggleSchema request;
        move_to_goal_ms.name = "move_to_goal_ms";
        move_to_goal_ms.value = true;
        request.request.schema.push_back(move_to_goal_ms); //allow robot to wander around randomly
        this->arc_base_client.call(request);

        this->currently_seeking_debris = true;
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
            this->currently_seeking_debris = false;
        }
    }

    //check to see once we are close enough to the debris location
    ROS_INFO_ONCE("Planning on going to position (%f, %f)", this->target_pose.position.x, this->target_pose.position.y);
}

void TaskUnguidedCleanDebrisServer::StateRemovingDebris() {
    //send request so we begin cleaning.

    if(!this->currently_cleaning) {
        dynamic_reconfigure::BoolParameter clean_debris_ms;
        arc_msgs::ToggleSchema request;
        clean_debris_ms.name = "clean_debris_ms";
        clean_debris_ms.value = true;
        request.request.schema.push_back(clean_debris_ms); //allow robot to wander around randomly
        this->arc_base_client.call(request);
        //we call the timer, once it's done the callback will be called, and next state is triggered.
        this->clean_debris_timer.setPeriod(ros::Duration(this->cleaning_debris_time));
        this->clean_debris_timer.start();
        this->currently_cleaning = true;
    }
}

void TaskUnguidedCleanDebrisServer::StateDoneDebrisRemoval() {
    this->debris_success_count++;
    this->debris_list.debris.erase(this->debris_list.debris.begin(), this->debris_list.debris.end());
    ROS_INFO("Succesfully cleaned debris. Total cleaned = %d", this->debris_success_count);
    this->target_debris = {}; //reset to defaults.

    //disable debris cleaner
    dynamic_reconfigure::BoolParameter clean_debris_ms;
    arc_msgs::ToggleSchema request;
    clean_debris_ms.name = "clean_debris_ms";
    clean_debris_ms.value = false;
    request.request.schema.push_back(clean_debris_ms); //allow robot to wander around randomly
    this->arc_base_client.call(request);

    this->currently_cleaning = false;

    ROS_ASSERT(!this->currently_cleaning);
    this->state = STATE_StartExploring;
}

void TaskUnguidedCleanDebrisServer::StateFailedDebrisRemoval() {
    ROS_INFO("Failed to clear debris.");
    dynamic_reconfigure::BoolParameter clean_debris_ms;
    arc_msgs::ToggleSchema request;
    clean_debris_ms.name = "clean_debris_ms";
    clean_debris_ms.value = false;
    request.request.schema.push_back(clean_debris_ms); //allow robot to wander around randomly
    this->arc_base_client.call(request);

    this->state = STATE_AbandonFailedDebris;
}

void TaskUnguidedCleanDebrisServer::StateAbandonFailedDebris() {
    //enable random wandering for some amount of time so we can escape debris
    const double ABANDON_DURATION = 5.0; //explore for 5 seconds when we can't clean a debris

    this->target_debris = {};
    if(!this->currently_abandoning) {
        dynamic_reconfigure::BoolParameter random_wander_ms;
        arc_msgs::ToggleSchema request;
        random_wander_ms.name = "random_wander_ms";
        random_wander_ms.value = true;
        request.request.schema.push_back(random_wander_ms); //allow robot to wander around randomly
        this->arc_base_client.call(request);
        this->abandon_failed_debris_timer.setPeriod(ros::Duration(ABANDON_DURATION));
        this->abandon_failed_debris_timer.start();
        this->currently_abandoning = true;
        this->currently_cleaning = false;
    }
}

void TaskUnguidedCleanDebrisServer::debris_locations_cb(const arc_msgs::DetectedDebris &debris) { this->debris_list = debris;
}
