#include "arc_msgs/ToggleSchema.h"
#include "TaskUnguidedCleanDebrisServer.h"

#define MAX_QUEUE_SIZE 1000

TaskUnguidedCleanDebrisServer::TaskUnguidedCleanDebrisServer() : server(global_handle, "task_explore", boost::bind(&TaskUnguidedCleanDebrisServer::goal_cb, this, _1), false)
{
    ros::NodeHandle local_handle("task_explore_server");
    ros::Timer timer = global_handle.createTimer(ros::Duration(60), &TaskUnguidedCleanDebrisServer::explore_timer_cb, this, false);
    this->debris_sub = global_handle.subscribe("detect_debris_ps/debris_locations", MAX_QUEUE_SIZE, &TaskUnguidedCleanDebrisServer::debris_locations_cb, this);
    this->base_pos_sub = global_handle.subscribe("base_pose_ground_truth", MAX_QUEUE_SIZE, &TaskUnguidedCleanDebrisServer::base_pose_cb, this);
    this->pose_listener;

    this->local_handle = local_handle;
    this->explore_timer = timer;
    //TODO: Handle pre-empt callback as well

    this->arc_base_client = global_handle.serviceClient<arc_msgs::ToggleSchema>("arc_base/toggle_schema");
    this->move_to_debris_client = global_handle.serviceClient<arc_msgs::NavigationRequest>("move_to_goal_ms/move_to_goal");

    //Enable the move_to_goal schema since we will be using it throughout task.
    dynamic_reconfigure::BoolParameter move_to_goal_ms;
    arc_msgs::ToggleSchema request;
    move_to_goal_ms.name = "move_to_goal_ms";
    move_to_goal_ms.value = true;
    request.request.schema.push_back(move_to_goal_ms); //allow robot to wander around randomly
    this->arc_base_client.call(request);
    this->server.start();
}

void TaskUnguidedCleanDebrisServer::base_pose_cb(const nav_msgs::Odometry &odom) {
    this->recent_pose = odom;
}

void TaskUnguidedCleanDebrisServer::goal_cb(const arc_msgs::ArcTaskGoalConstPtr &goal) {
    ROS_INFO("Executing task_explore");

    this->startup();
    this->process();
}

void TaskUnguidedCleanDebrisServer::explore_timer_cb(const ros::TimerEvent &event) {
    //State transition
    ROS_INFO("Explore timer cb called. Started at %d and ended at %d",event.last_real, event.current_real);
}

void TaskUnguidedCleanDebrisServer::shutdown() {
    arc_msgs::ToggleSchema request;

    dynamic_reconfigure::BoolParameter random_wander_ms;
    random_wander_ms.name = "random_wander_ms";
    random_wander_ms.value = false;
    request.request.schema.push_back(random_wander_ms); //allow robot to wander around randomly

    this->arc_base_client.call(request);
}

void TaskUnguidedCleanDebrisServer::startup() {
    this->state = STATE_StartExploring;
    ROS_INFO("Starting up task_explore");
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
            ROS_INFO_ONCE("IN STATE: SeekingDebrisLocation.");
            StateSeekingDebrisLocation();
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
    static bool hunting_debris = false; //False if we are not yet actively hunting for the debris

    //only send a goal if we have not sent one already
    if(!hunting_debris) {
        //Enable the move_to_goal schema so we can move to the debris
        dynamic_reconfigure::BoolParameter move_to_goal_ms;
        arc_msgs::ToggleSchema request;
        move_to_goal_ms.name = "move_to_goal_ms";
        move_to_goal_ms.value = true;
        request.request.schema.push_back(move_to_goal_ms); //allow robot to wander around randomly
        this->arc_base_client.call(request);

        hunting_debris = true;
        arc_msgs::NavigationRequest req;
        req.request.pose.position.x = this->target_pose.position.x;
        req.request.pose.position.y = this->target_pose.position.y;

        ROS_INFO("Sending navigation request to location (%f, %f)", req.request.pose.position.x, req.request.pose.position.y);
        this->move_to_debris_client.call(req);
    }

    //check to see once we are close enough to the debris location

    ROS_INFO_ONCE("Planning on going to position (%f, %f)", this->target_pose.position.x, this->target_pose.position.y);
}

void TaskUnguidedCleanDebrisServer::StateRemovingDebris() {

}

void TaskUnguidedCleanDebrisServer::StateDoneDebrisRemoval() {

}

void TaskUnguidedCleanDebrisServer::StateFailedDebrisRemoval() {

}

void TaskUnguidedCleanDebrisServer::StateAbandonFailedDebris() {

}

void TaskUnguidedCleanDebrisServer::debris_locations_cb(const arc_msgs::DetectedDebris &debris) {
    if (debris.debris.size() > 0) {
        this->debris_list = debris;
    }
}
