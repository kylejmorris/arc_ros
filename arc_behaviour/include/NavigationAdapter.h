/**
* CLASS: NavigationAdaptor
* DATE: 16/02/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Provides adapted interface between arc_base and ros navigation stack.
* Allow for filtering navigation requests based on priority. For example... when randomly wandering,
 * multiple navigation requests may be sent to the navigation stack. When a request to move to a robot is sent,
 * this will have a higher priority, and will not be interrupted by other requests. This ensures important navigation is completed.
*/

#ifndef PROJECT_NAVIGATIONADAPTER_H
#define PROJECT_NAVIGATIONADAPTER_H

#include <ros/ros.h>
#include "std_srvs/Empty.h"
#include <geometry_msgs/Pose.h>
#include "arc_msgs/NavigationRequest.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_srvs/Trigger.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
namespace arc_behaviour {
    class NavigationAdapter{
    private:
        const int DEFAULT_NAVIGATION_PRIORITY = 0;
        ros::NodeHandle global_handle;

        ros::NodeHandle local_handle;

        /*
         * Any requests to move to some location will be checked here before calling upon navigation stack.
         */
        ros::ServiceServer move_to_goal_server;

        /**
         * Used to determine if robot is currently stuck (ie it won't make it to any navigation goals)
         */
        ros::ServiceServer is_stuck_server;
        /**
         * Our communication with the navigation stack.
         */
        MoveBaseClient *move_client;

        /**
         * Whether or not the navigation stack is currently being used to reach a goal.
         */
        bool goal_active;
        /**
         * When goal is active, we have some priority that was specified with the navigation request.
         * If any further requests are made to change the navigation, the priority must be higher than the value specified here.
         */
        int current_nav_priority;

        bool is_stuck = false; //set to true if we tried reaching goal but received fail/stuck signal back from navigation stack.

        /**
         * Process navigation request and send it to be handled by navigation stack.
         * @param req: The accepted navigation request.
         */
        void sendGoal(arc_msgs::NavigationRequest::Request &req);
    public:
        NavigationAdapter();

        /**
         * Move to navigation target location.
         */
        bool move_to_goal_request_cb(arc_msgs::NavigationRequest::Request &req, arc_msgs::NavigationRequest::Response &res);

        /**
         * Check if robot is stuck currently.
         * @return True if stuck, false otherwise.
         */
        bool is_stuck_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        /**
         * When move_base is done, it will send a callback to signal the result of navigation. This may be either when the navigation is completed
         * succesfully or it fails to navigate.
         */
        void move_to_goal_result_cb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);

        bool isGoal_active() const;

        void setGoal_active(bool goal_active);

        int getCurrent_nav_priority() const;

        void setCurrent_nav_priority(int current_nav_priority);

        bool isIs_stuck() const;

        void setIs_stuck(bool is_stuck);

        //run the main loop of node.
        void run();
    };
}

#endif //PROJECT_NAVIGATIONADAPTOR_H
