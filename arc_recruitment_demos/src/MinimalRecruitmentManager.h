/**
* CLASS: MinimalRecruitmentManager
* DATE: 13/04/17
* AUTHOR: Kyle Morris
* DESCRIPTION: A bare-bones recruitment manager that just listens for tasks and then performs them, nothing much more.
*/

#ifndef ARC_RECRUITMENT_DEMOS_MINIMALRECRUITMENTMANAGER_H
#define ARC_RECRUITMENT_DEMOS_MINIMALRECRUITMENTMANAGER_H
#include <ros/ros.h>
#include "arc_msgs/TaskRequest.h"

class MinimalRecruitmentManager {
private:
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle;

    /**
     * Subscribe to incoming task requests.
     */
    ros::Subscriber task_requests_sub;

    /**
     * Tasks we've accepted and are publishing to the task_handler to activate;
     */
    ros::Publisher task_request_pub;


public:
    MinimalRecruitmentManager();

    /**
     * The main loop;
     */
    void process();

    void task_request_cb(const arc_msgs::TaskRequest &req);
};

#endif //ARC_RECRUITMENT_DEMOS_MINIMALRECRUITMENTMANAGER_H
