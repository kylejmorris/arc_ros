/**
* CLASS: MoveToGoalMS
* DATE: 31/03/2017
* AUTHOR: Kyle Morris
* DESCRIPTION: Sends direct goals to the navigation stack, moves with intent.
*/
#ifndef ARC_BEHAVIOUR_MOVETOGOALMS_H
#define ARC_BEHAVIOUR_MOVETOGOALMS_H

#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include "arc_msgs/NavigationRequest.h"
#include "MotorSchema.h"
#include "std_srvs/SetBool.h"

namespace arc_behaviour {
    class MoveToGoalMS : public MotorSchema {
    private:
        int DEFAULT_PRIORITY = 2;
        std::string DEFAULT_FRAME_ID = "map";
        /*
         * Client to randomly send requests for navigation to navigation stack.
         */
        ros::ServiceClient move_to_goal_client;

        /**
         * Send requests to cancel robots navigation
         */
        ros::ServiceClient abort_goals_client;

        /**
         * Service allowing enabling/disabling of this schema.
         */
        ros::ServiceServer toggle_server;

        /**
         * Allow us to move to a specific goal in the world.
         */
        ros::ServiceServer move_to_goal_server;

        ros::NodeHandle global_handle;
        ros::NodeHandle local_handle;

        ros::Subscriber base_pose_sub;

        /**
         * how much priority this behaviour has. Will determine if it is accepted by navigation adapter.
         */
        int priority;

        /**
         * Map dimensions are required when we are using global map frame to generate coordinates.
         */
        int map_width = -1;
        int map_height = -1;

        void toggle(bool state);
    public:
        MoveToGoalMS();

        void run();

        bool toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
        bool move_to_goal_cb(arc_msgs::NavigationRequest::Request &req, arc_msgs::NavigationRequest::Response &res);
    };
}
#endif //ARC_BEHAVIOUR_RANDOMWANDERMS_H
