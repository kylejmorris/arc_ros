/**
* CLASS: TaskExploreClient
* DATE: 28/03/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Client for toggling explore task
*/
#include "ros/ros.h"
#include <ros/ros.h>
#include "arc_msgs/ArcTaskAction.h"
#include <actionlib/client/simple_action_client.h>

#ifndef ARC_TASKS_TASKEXPLORECLIENT_H
#define ARC_TASKS_TASKEXPLORECLIENT_H

class TaskExploreClient {
    typedef actionlib::SimpleActionClient<arc_msgs::ArcTaskAction> ActionClient;
private:
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle;

    ActionClient *client;

    ros::ServiceClient arc_base_client;

public:
    TaskExploreClient();

    /**
     * Calling this when task is complete
     */
    void result_cb(const actionlib::SimpleClientGoalState& state, const arc_msgs::ArcTaskResultConstPtr &result);

    void doTask(const ros::TimerEvent &event);
};
#endif //ARC_TASKS_TASKEXPLORECLIENT_H
