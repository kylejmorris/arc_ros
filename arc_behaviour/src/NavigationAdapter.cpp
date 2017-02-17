#include "NavigationAdapter.h"
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
}

void NavigationAdapter::move_to_goal_result_cb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    if (state == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO("Failed to move");
    }else if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("successfully moved to goal!!!");
    }

    //either way, we are done navigating (until another request is sent)
    this->goal_active = false;
}

bool NavigationAdapter::move_to_goal_request_cb(arc_msgs::NavigationRequest::Request &req, arc_msgs::NavigationRequest::Response &res) {
    ROS_INFO("Received request to move to goal (%d, %d) with priority %d", req.pose.position.x, req.pose.position.y, req.priority);

    move_base_msgs::MoveBaseGoal goal;
    if(this->goal_active) {
    } else {
        //If no goal is currently active, we can send request to move_base
        //we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = req.pose.position.x;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending navigation goal");
        this->move_client->sendGoal(goal, boost::bind(&NavigationAdapter::move_to_goal_result_cb, this,_1,_2));
        return true;
    }
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
