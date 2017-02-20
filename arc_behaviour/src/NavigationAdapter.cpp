#include "../include/NavigationAdapter.h"
#include <actionlib/client/simple_action_client.h>
#include "arc_msgs/NavigationRequest.h"
#include "std_srvs/Empty.h"

using namespace arc_behaviour;

NavigationAdapter::NavigationAdapter() {
    ros::NodeHandle nh;
    this->global_handle = nh;
    ros::NodeHandle local_handle("navigation_adapter");
    this->local_handle = local_handle;
    ROS_INFO("Setting up navigation adapter..");
    this->move_to_goal_server = local_handle.advertiseService("move_to_goal",&NavigationAdapter::move_to_goal_request_cb, this);
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
    }else if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("successfully moved to goal!!!");
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
        assert(this->current_nav_priority >= this->DEFAULT_NAVIGATION_PRIORITY);

        //Let higher priority request take control of navigation stack.
        if(req.priority > this->current_nav_priority) {
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
}

void NavigationAdapter::run() {
    ros::Rate r(10); //TODO: Make this a parameter, just get rid of magic numbers.

    while(ros::ok()) {
        /*if(this->move_client.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
            ROS_INFO("Navigation goal is active.");
        } else {
            ROS_INFO("navigation is not active");
        }*/
        ros::spinOnce();
        r.sleep();
    }
}
