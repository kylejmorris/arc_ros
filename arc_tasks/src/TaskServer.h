/**
* CLASS: TaskServer
* DATE: 06/04/17
* AUTHOR: Kyle Morris
* DESCRIPTION: All task servers must provide this functionality
*/

#ifndef ARC_TASKS_TASKSERVER_H
#define ARC_TASKS_TASKSERVER_H
#include "arc_msgs/ArcTaskAction.h"

class TaskServer {
private:
    /**
     * Perform any routine startup procedures when this task instance is started.
     * Load request parameters, check for existence of nodes needed for this task.
     * @parameter goal: The goal set for this task
     */
    virtual void startup(const arc_msgs::ArcTaskGoalConstPtr &goal) = 0;

    /**
     * The main state machine loop for the task.
     */
    virtual void process() = 0;

    /**
     * Ensure after this task instance is no longer of use, we've shut down everything that was required for it.
     */
    virtual void shutdown() = 0;

public:
     /**
     * Receives request to perform the task.
     */
    void goal_cb(const arc_msgs::ArcTaskGoalConstPtr &goal);
};

#endif //ARC_TASKS_TASKSERVER_H
