/**
* CLASS: TaskGuidedCleanDebrisClient
* DATE: 4/04/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Client for GuidedCleanDebris task
*/
#ifndef ARC_TASKS_TASKGUIDEDCLEANDEBRISCLIENT_H
#define ARC_TASKS_TASKGUIDEDCLEANDEBRISCLIENT_H
#include "ros/ros.h"
#include <ros/ros.h>
#include "arc_msgs/ArcTaskAction.h"
#include <actionlib/client/simple_action_client.h>

class TaskGuidedCleanDebrisClient {
    typedef actionlib::SimpleActionClient<arc_msgs::ArcTaskAction> ActionClient;
private:
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle;

    ActionClient *client;

    ros::ServiceClient arc_base_client;

public:
    TaskGuidedCleanDebrisClient();

    /**
     * Calling this when task is complete
     */
    void result_cb(const actionlib::SimpleClientGoalState& state, const arc_msgs::ArcTaskResultConstPtr &result);

    void doTask(const ros::TimerEvent &event);
};

#endif //ARC_TASKS_TASKUNGUIDEDCLEANDEBRISCLIENT_H
