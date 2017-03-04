#include "RandomWanderMS.h"
#include "arc_msgs/NavigationRequest.h"
#define MAX_QUEUE_SIZE 1000

using namespace arc_behaviour;

RandomWanderMS::RandomWanderMS() {
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle("random_wander_ms");
    this->global_handle = global_handle;
    this->local_handle = local_handle;
    ROS_INFO("Setting up random wander ms");
    local_handle.param("max_range", this->max_range, this->DEFAULT_MAX_RANGE);
    local_handle.param("update_goal_feq", this->random_choice_rate, this->DEFAULT_RANDOM_CHOICE_RATE);
    local_handle.param("frame_id", this->frame_id, this->DEFAULT_FRAME_ID);
    this->move_to_goal_client = this->global_handle.serviceClient<arc_msgs::NavigationRequest>("navigation_adapter/move_to_goal");
    this->toggle_server = this->local_handle.advertiseService("toggle", &RandomWanderMS::toggle_cb, this);
    this->base_pose_sub = this->global_handle.subscribe("base_pose_ground_truth", MAX_QUEUE_SIZE, &RandomWanderMS::process_base_pose_cb, this);
    this->priority = local_handle.getParam("priority", this->DEFAULT_PRIORITY);
    ROS_INFO("Parameter max_range set: %f", this->max_range);
    ROS_INFO("Parameter update_goal_greq set: %f", this->random_choice_rate);
    ROS_INFO("Parameter frame_id set: %s", this->frame_id.c_str());

    if(this->frame_id=="map") {
        //we need to get map width and height
        global_handle.param("move_base_node/global_costmap/height", this->map_height, -1);
        global_handle.param("move_base_node/global_costmap/width", this->map_width, -1);

        if(this->map_height < 0 || this->map_width < 0) {
            ROS_ERROR("Set RandomWanderMS to use map frame; but could not determine map dimensions");
        } else {

            ROS_INFO("RandomWanderMS acquired map dimensions (%d, %d)", this->map_width, this->map_height);
        }
    } else {
        ROS_INFO("Frame of reference is set to %s", this->frame_id.c_str());
    }
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

void RandomWanderMS::process_base_pose_cb(nav_msgs::Odometry odom) {
    this->recent_position = odom;
}

arc_msgs::NavigationRequest RandomWanderMS::generateRequest() {
    arc_msgs::NavigationRequest req;
    const double BUFFER = 0.5; //we don't want to go any closer than this to the edge of map
    assert(this->max_range>0);
    double x;
    double y;


    req.request.priority = this->priority;

    if(this->frame_id=="base_link") {
        //points are relative to robots current position
        double delta_x = (rand() % (int)this->max_range) - ((int)this->max_range)/2.0; //TODO: If maxrange is a double, this will truncate some accuracy. Fix the mod op to support double value for max_range
        double delta_y = (rand() % (int)this->max_range) - ((int)this->max_range)/2.0; //TODO: Test this random generation, make sure it's proper.
        double current_x = this->recent_position.pose.pose.position.x;
        double current_y = this->recent_position.pose.pose.position.y;

        x = current_x + delta_x;
        y = current_y + delta_y;

        //If our planned points go out of the world, we need to adjust them to be nearest point,
        //that is within our map.
        /*if(current_x + delta_x < BUFFER) {
            x = BUFFER;
        }
        if(current_x + delta_x > this->map_width - BUFFER) {
            x = this->map_width - BUFFER;
        }
        if(current_x + delta_x < BUFFER) {
            y = BUFFER;
        }
        if(current_y + delta_y > this->map_width - BUFFER) {
            y = this->map_height - BUFFER;
        } */
        ROS_INFO("using base_link coordinate frame");
    } else if(this->frame_id=="map") {
        //TODO: Test these and ensure they are random
        //points are converted into map coordinates directly.
        x = (rand() % (this->map_width));
        y = (rand() % (this->map_height));
    }

    ROS_INFO("Generated random navigation request(%d, %d) ", (int)x, (int)y);
    req.request.pose.position.x  = x;
    req.request.pose.position.y  = y;
    req.request.pose.position.z = 0;
    req.request.pose.orientation.w  = 1.0; //default req.request.pose.orientation.x  = 0; //default req.request.pose.orientation.y  = 0; //default req.request.pose.orientation.z  = 0; //default return req;
}
