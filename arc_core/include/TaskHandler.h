/**
* CLASS: TaskHandler
* DATE: 07/04/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Similar to arc_base, in that this node allows for toggling of tasks
* to complete.
* There are restrictions on what combinations of tasks can be done at the same time.
*
* The task handler is a client with respect to task servers it communicates with, however it is a server
* with respect to the Recruitment Manager
*
* For example, while multiple background tasks may be done together, you cannot do
* multiple non-background tasks at the same time.
*/

#ifndef ARC_TASKS_TASKHANDLER_H
#define ARC_TASKS_TASKHANDLER_H
#include "arc_msgs/ArcTaskHandlerAction.h"
#include "arc_msgs/ArcTaskAction.h"
#include "arc_msgs/TaskRequest.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <map>
#include <string>

class TaskHandler {
    typedef actionlib::SimpleActionClient<arc_msgs::ArcTaskAction> ActionClient;
    typedef arc_msgs::TaskRequest TaskGoal;

private:
    /**
     * Subscribes to incoming task requests
     */
    ros::Subscriber task_request_sub;

    //Handles for this node
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle;

    /**
     * Mapping of all the task names to their designated clients.
     */
    std::map<std::string, ActionClient *> task_clients;

    /**
     * List of the task requests we've been given but have not started. If we are currently in the middle of a request, then we put this here.
     */
    std::vector<TaskGoal> task_backlog;

    /**
     * The tasks currently requested and being worked on. When a task is done we remove it from this list
     */
    std::vector<TaskGoal> active_tasks;

    /**
     * Upon receiving a task request, process it here.
     * @param goal The tasks requested
     */
    void processRequest(const TaskGoal &goal);

    /**
     * When a request is rejected we publish the rejection information and record that this occured.
     * @param goal: The goal request that was rejected
     */
    void rejectRequest(const TaskGoal goal);
    /**
     * When a request is accepted, we begin the task and keep track that it is active.
     * @param goal: The task request that was accepted
     */
    void acceptRequest(const TaskGoal goal);

    void startupActionClients();
public:
    TaskHandler();

    /**
     * The main loop waiting for callbacks.
     */
    void process();

    /**
     * Determine if the request to the TaskHandler is doable, that is, it can be done at the same time as other
     * tasks.
     * can be completed at the same time, and have valid information provided.
     *
     * @param goal: The requests of all the tasks to work on.
     * @return True if the request is valid and these tasks will be activated. False otherwise.
     */
    bool isAcceptableRequest(const TaskGoal &goal);

//SERVER: Routines with the perspective that TaskHandler is a server with respect to RecruitmentManager
    /**
     * Requests to the TaskHandler go here. This will sift through them and activate the individual tasks.
     *
     * @param goal The request sent to the task handler
     */
    void task_handler_goal_cb(const TaskGoal &goal);

//CLIENT: Routines with the perspective that TaskHandler is a client with respect to TaskServer's
    /**
     * Called whenever a task is done execution this will be called.
     * @param state: The state of goal when we hear back.
     * @param result: Information about what the task completed.
     */
    void task_handler_result_cb(const actionlib::SimpleClientGoalState& state, const arc_msgs::ArcTaskResultConstPtr &result);
};

#endif //ARC_TASKS_TASKHANDLER_H
