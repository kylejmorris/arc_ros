/**
* CLASS: TaskUnguidedCleanDebrisClient
* DATE: 29/03/17
* AUTHOR: ${AUTHOR}
* DESCRIPTION: ${DESCRIPTION}
*/
#ifndef ARC_TASKS_TASKUNGUIDEDCLEANDEBRISCLIENT_H
#define ARC_TASKS_TASKUNGUIDEDCLEANDEBRISCLIENT_H
#include "ros/ros.h"
#include <ros/ros.h>
#include "arc_msgs/ArcTaskAction.h"
#include <actionlib/client/simple_action_client.h>

class TaskUnguidedCleanDebrisClient {
    typedef actionlib::SimpleActionClient<arc_msgs::ArcTaskAction> ActionClient;
private:
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle;

    ActionClient *client;

    ros::ServiceClient arc_base_client;

public:
    TaskUnguidedCleanDebrisClient();

    /**
     * Calling this when task is complete
     */
    void result_cb(const actionlib::SimpleClientGoalState& state, const arc_msgs::ArcTaskResultConstPtr &result);

    void doTask(const ros::TimerEvent &event);
};

#endif //ARC_TASKS_TASKUNGUIDEDCLEANDEBRISCLIENT_H
