#include "../include/NavigationAdapter.h"
#include <actionlib/client/simple_action_client.h>
#include "arc_msgs/NavigationRequest.h"
#include "std_srvs/Empty.h"

using namespace arc_behaviour;
bool NavigationAdapter::is_stuck_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    return this->is_stuck;
}


NavigationAdapter::NavigationAdapter() {
    ros::NodeHandle nh;
    ros::NodeHandle local_handle("navigation_adapter");

    this->global_handle = nh;
    this->local_handle = local_handle;
    this->is_stuck = false;

    ROS_INFO("Setting up navigation adapter..");

    this->move_to_goal_server = local_handle.advertiseService("move_to_goal",&NavigationAdapter::move_to_goal_request_cb, this);
    this->is_stuck_server = local_handle.advertiseService("is_stuck",&NavigationAdapter::is_stuck_cb, this);
    this->move_client = new MoveBaseClient("move_base", true);

    //wait for the action server to come up
    while(!move_client->waitForServer(ros::Duration(5.0))) { //TODO: Better checking to ensure action server is actually up. If it's not, we need to let user know.
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ROS_INFO("Found move_base action server!");
    this->current_nav_priority = 0;
}

void NavigationAdapter::move_to_goal_result_cb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    if (state == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO("Failed to move");
        //we just assume that if the robot couldn't move, and the goal was aborted because of this, we are stuck.
        this->is_stuck = true;
    }else if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("successfully moved to goal!!!");
        this->is_stuck = false; //not stuck if you just reached a goal.
    } else {
        ROS_WARN("Unexpected result from move_to_goal_result_cb");
    }

    //either way, we are done navigating (until another request is sent)
    this->current_nav_priority = this->DEFAULT_NAVIGATION_PRIORITY;
    this->goal_active = false;
}
//TODO: Define a valid priority range, and ensure requests are within that range. If message is sent without priority specified, it wil be 412412 or something high, and will automatically go through...
bool NavigationAdapter::move_to_goal_request_cb(arc_msgs::NavigationRequest::Request &req, arc_msgs::NavigationRequest::Response &res) {
    ROS_INFO("Received request to move to goal (%d, %d) with priority %d", req.pose.position.x, req.pose.position.y, req.priority);

    move_base_msgs::MoveBaseGoal goal;
    if(this->goal_active) {
        assert(this->current_nav_priority >= this->DEFAULT_NAVIGATION_PRIORITY); //TODO: why using assert?

        //Let higher priority request take control of navigation stack.
        if(req.priority >= this->current_nav_priority) {
            this->sendGoal(req);
            return true;
        } else {
            ROS_INFO("Navigation request rejected. Request priority (%d) < active priority (%d)", req.priority, this->current_nav_priority);
        }
    } else {
        //If no goal is currently active, we can send request to move_base
        this->sendGoal(req);
        return true;
    }
}

void NavigationAdapter::sendGoal(arc_msgs::NavigationRequest::Request &req) {
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    //setting goal information from request.
    goal.target_pose.pose.position.x = req.pose.position.x;
    goal.target_pose.pose.position.y = req.pose.position.y;
    goal.target_pose.pose.position.z = req.pose.position.z;
    goal.target_pose.pose.orientation.w = req.pose.orientation.w;
    goal.target_pose.pose.orientation.x = req.pose.orientation.x;
    goal.target_pose.pose.orientation.y = req.pose.orientation.y;
    goal.target_pose.pose.orientation.z = req.pose.orientation.z;

    //TODO: Add parameters to control max values that can be set here
    ROS_INFO("Sending navigation goal");
    this->move_client->sendGoal(goal, boost::bind(&NavigationAdapter::move_to_goal_result_cb, this,_1,_2));
    this->current_nav_priority = req.priority;
    this->goal_active = true;
    this->is_stuck = false; //we aren't stuck at the moment we send a goal
}

void NavigationAdapter::run() {
    ros::Rate r(10); //TODO: Make this a parameter, just get rid of magic numbers.

    while(ros::ok()) {
        if(this->is_stuck) {
            ROS_INFO("ROBOT IS STUCK");
        }
        ros::spinOnce();
        r.sleep();
    }
}

bool NavigationAdapter::isGoal_active() const {
    return goal_active;
}

void NavigationAdapter::setGoal_active(bool goal_active) {
    NavigationAdapter::goal_active = goal_active;
}

int NavigationAdapter::getCurrent_nav_priority() const {
    return current_nav_priority;
}

void NavigationAdapter::setCurrent_nav_priority(int current_nav_priority) {
    NavigationAdapter::current_nav_priority = current_nav_priority;
}

bool NavigationAdapter::isIs_stuck() const {
    return is_stuck;
}

void NavigationAdapter::setIs_stuck(bool is_stuck) {
    NavigationAdapter::is_stuck = is_stuck;
}
