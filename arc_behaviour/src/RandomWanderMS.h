/**
* CLASS: RandomWanderMS
* DATE: 20/02/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Random wander behaviour that sends various random navigation goals to navigation stack.
*/
#ifndef ARC_BEHAVIOUR_RANDOMWANDERMS_H
#define ARC_BEHAVIOUR_RANDOMWANDERMS_H
#include "ros/ros.h"
#include "arc_msgs/NavigationRequest.h"

namespace arc_behaviour {
    class RandomWanderMS {
    private:
        double DEFAULT_RANDOM_CHOICE_RATE = 0.1; //update every 10 second TODO: Read this as parameter
        int DEFAULT_PRIORITY = 1;
        double DEFAULT_MAX_RANGE = 10.0;
        /*
         * Client to randomly send requests for navigation to navigation stack.
         */
        ros::ServiceClient move_to_goal_client;

        ros::NodeHandle global_handle;
        ros::NodeHandle local_handle;

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
         * Generate a random navigation request
         * @return NavigationRequest generated.
         */
        arc_msgs::NavigationRequest generateRequest();

        void setMaxRange(double max_range);


    public:
        RandomWanderMS();

        void run();
    };
}

#endif //ARC_BEHAVIOUR_RANDOMWANDERMS_H
