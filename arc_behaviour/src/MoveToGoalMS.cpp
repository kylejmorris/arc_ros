#include <std_srvs/Trigger.h>
#include "MoveToGoalMS.h"
#include "arc_msgs/NavigationRequest.h"

#define MAX_QUEUE_SIZE 1000
#define DEFAULT_RATE 10

using namespace arc_behaviour;

MoveToGoalMS::MoveToGoalMS() {
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle("move_to_goal_ms");

    this->global_handle = global_handle;
    this->local_handle = local_handle;
    ROS_INFO("Setting up move_to_goal ms");
    this->move_to_goal_client = this->global_handle.serviceClient<arc_msgs::NavigationRequest>("navigation_adapter/move_to_goal");
    this->abort_goals_client = this->global_handle.serviceClient<std_srvs::Trigger>("navigation_adapter/abort_goals");
    this->toggle_server = this->local_handle.advertiseService("toggle", &MoveToGoalMS::toggle_cb, this);
    this->move_to_goal_server = this->local_handle.advertiseService("move_to_goal", &MoveToGoalMS::move_to_goal_cb, this);
    this->priority = local_handle.getParam("priority", this->DEFAULT_PRIORITY);

    //starts off disabled.
    this->toggle(false);
}

bool MoveToGoalMS::move_to_goal_cb(arc_msgs::NavigationRequest::Request &req, arc_msgs::NavigationRequest::Response &res) {
    if(this->enabled) {
        //TODO: Check to ensure goal is within map boundaries
        arc_msgs::NavigationRequest request;
        request.request.pose.position.x = req.pose.position.x;
        request.request.pose.position.y = req.pose.position.y;
        request.request.priority = this->priority;
        request.request.pose.orientation.w = 1.0;
        this->move_to_goal_client.call(request);
    }

    return true;
}

void MoveToGoalMS::run() {
    ros::Rate r(DEFAULT_RATE);
    //set a timer to call random generation of goals every once in a while

    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

}

bool MoveToGoalMS::toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    this->toggle(req.data);
    return true;
}

void MoveToGoalMS::toggle(bool state) {
    this->enabled = state;
    if(this->enabled) {
        ROS_INFO("MoveToGoalMS has been enabled.");
        //send an immediate goal right when we start.
    } else {
        ROS_INFO("MoveToGoalMS has been disabled.");
        std_srvs::Trigger req;
        this->abort_goals_client.call(req);
    }
}
