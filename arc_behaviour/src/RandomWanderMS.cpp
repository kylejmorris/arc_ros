#include "RandomWanderMS.h"
#include "arc_msgs/NavigationRequest.h"

using namespace arc_behaviour;

RandomWanderMS::RandomWanderMS() {
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle("random_wander_ms");
    this->global_handle = global_handle;
    this->local_handle = local_handle;
    ROS_INFO("Setting up random wander ms");
    local_handle.param("max_range", this->max_range, this->DEFAULT_MAX_RANGE);
    local_handle.param("update_goal_feq", this->random_choice_rate, this->DEFAULT_RANDOM_CHOICE_RATE);
    this->move_to_goal_client = this->global_handle.serviceClient<arc_msgs::NavigationRequest>("navigation_adapter/move_to_goal");
    this->toggle_server = this->local_handle.advertiseService("toggle", &RandomWanderMS::toggle_cb, this);
    this->priority = local_handle.getParam("priority", this->DEFAULT_PRIORITY);
    ROS_INFO("Parameter max_range set: %f", this->max_range);
    ROS_INFO("Parameter update_goal_greq set: %f", this->random_choice_rate);

    //starts off disabled.
    this->toggle(false);
}

void RandomWanderMS::setMaxRange(double max_range) {
    if(max_range<=0) {
        ROS_WARN("Unable to set parameter: max_range. Value must be > 0. Using default.");
        this->max_range = this->DEFAULT_MAX_RANGE;
    } else {
        this->max_range = max_range;
    }
}

void RandomWanderMS::run() {
    ros::Rate r(this->random_choice_rate);

    while(ros::ok()) {
        //only do stuff if schema is enabled.
        if(this->enabled) {
            arc_msgs::NavigationRequest req = this->generateRequest();
            this->move_to_goal_client.call(req);

            ROS_INFO("sent random wander goal of (%d, %d)", (int) req.request.pose.position.x,
                     (int) req.request.pose.position.y);

        }

        ros::spinOnce();
        r.sleep();
    }

}

bool RandomWanderMS::toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    this->toggle(req.data);
    return true;
}

void RandomWanderMS::toggle(bool state) {
    this->enabled = state;
    if(this->enabled) {
        ROS_INFO("RandomWanderMS has been enabled.");
    } else {
        ROS_INFO("RandomWanderMS has been disabled.");
    }
}

arc_msgs::NavigationRequest RandomWanderMS::generateRequest() {
    arc_msgs::NavigationRequest req;
    assert(this->max_range>0);
    double x = (rand() % (int)this->max_range) - ((int)this->max_range)/2.0; //TODO: If maxrange is a double, this will truncate some accuracy. Fix the mod op to support double value for max_range
    double y = (rand() % (int)this->max_range) - ((int)this->max_range)/2.0; //TODO: Test this random generation, make sure it's proper.

    ROS_INFO("Generated (%d, %d) ", (int)x, (int)y);

    req.request.priority = this->priority;
    req.request.pose.position.x = x;
    req.request.pose.position.y  = y;
    req.request.pose.position.z = 0;
    req.request.pose.orientation.w  = 1.0; //default
    req.request.pose.orientation.x  = 0; //default
    req.request.pose.orientation.y  = 0; //default
    req.request.pose.orientation.z  = 0; //default

    return req;
}
