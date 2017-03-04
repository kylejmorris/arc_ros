/**
* CLASS: RandomWanderMS
* DATE: 20/02/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Random wander behaviour that sends various random navigation goals to navigation stack.
*/
#ifndef ARC_BEHAVIOUR_RANDOMWANDERMS_H
#define ARC_BEHAVIOUR_RANDOMWANDERMS_H

#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include "arc_msgs/NavigationRequest.h"
#include "MotorSchema.h"
#include "std_srvs/SetBool.h"

namespace arc_behaviour {
    class RandomWanderMS : public MotorSchema {
    private:
        double DEFAULT_RANDOM_CHOICE_RATE = 0.1; //update every 10 second
        int DEFAULT_PRIORITY = 1;
        double DEFAULT_MAX_RANGE = 10.0;
        std::string DEFAULT_FRAME_ID = "map";
        /*
         * Client to randomly send requests for navigation to navigation stack.
         */
        ros::ServiceClient move_to_goal_client;

        /**
         * Service allowing enabling/disabling of this schema.
         */
        ros::ServiceServer toggle_server;

        ros::NodeHandle global_handle;
        ros::NodeHandle local_handle;

        ros::Subscriber base_pose_sub;

        /**
         * Keeping track of most recent position, so we can drop markers relative to our position;
         */
        nav_msgs::Odometry recent_position;
        /**
         * How often (in hz) we will update the random goal.
         */
        double random_choice_rate;

        /**
         * how much priority this behaviour has. Will determine if it is accepted by navigation adapter.
         */
        int priority;

        /**
         * Maximum distance in meters, that a goal can be. Relative to current location.
         */
        double max_range;

        /**
         * Map dimensions are required when we are using global map frame to generate coordinates.
         */
        int map_width = -1;
        int map_height = -1;

        /**
         * The frame points are generated in. /map for example, means relative to global map.
         * /base_link would indicate any point (x,y) is this distance relative to robots location.
         */
        std::string frame_id;
        /**
         * Generate a random navigation request
         * @return NavigationRequest generated.
         */
        arc_msgs::NavigationRequest generateRequest();

        void setMaxRange(double max_range);

        void toggle(bool state);
    public:
        RandomWanderMS();

        void run();

        bool toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
        void process_base_pose_cb(nav_msgs::Odometry odom);
    };
}

#endif //ARC_BEHAVIOUR_RANDOMWANDERMS_H
