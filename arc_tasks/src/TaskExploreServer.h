/**
* CLASS: TaskExplore
* DATE: 23/03/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Activates the random wander motor schema to navigate around a bit.
* The simplest of tasks, if you choose to accept it.
*/
#include <ros/ros.h>
#include "arc_msgs/ArcTaskAction.h"
#include <actionlib/server/simple_action_server.h>

#ifndef ARC_TASKS_TASKEXPLORE_H
#define ARC_TASKS_TASKEXPLORE_H
typedef actionlib::SimpleActionServer<arc_msgs::ArcTaskAction> ActionServer;

class TaskExploreServer {
private:
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle;

    ActionServer server;

    /**
     * Timer to keep track of how much longer we explore the environment.
     * Once started, it will continuosly cycle through a specified countdown and then call the
     * explore_timer_cb
     */
    ros::Timer explore_timer;

    /**
     * used to call the arc_base node and toggle the schema of our choice
     */
    ros::ServiceClient arc_base_client;

    //control the state of the debris exploration/removal
    typedef enum {
        STATE_StartExploring, //Enable schemas and begin exploring behaviour
        STATE_Exploring, //We are currently exploring
        STATE_DoneExploring //time has run out and we no longer need to explore
    } State;
    State state;

    /**
     * The results of running this task. Nothing required for the explore task.
     */
    arc_msgs::ArcTaskResult result;

    /**
     * Whether or not the task is currently active.
     */
    bool active = false;

public:
    TaskExploreServer();

    /**
     * perform any routine startup procedures when this task instance is started.
     * Load request parameters, check for existence of nodes needed for this task.
     */
    void startup(const arc_msgs::ArcTaskGoalConstPtr &goal);

    /**
     * Ensure after this task instance is no longer of use, we've shut down everything that was required for it.
     */
    void shutdown();
    /**
     * Perform the shutdown for a task, disabling anything that was originally set to the complete it.
     */
    void StateStartExploring();

    /**
     * Startup the task, enabling anything that takes place in it's task instance.
     */
    void StateExploring();
    void StateDoneExploring();

    /**
     * the main state machine loop for the task
     */
    void process();

    /**
     * Perform the main task. Explore for some amount of time
     */
    void goal_cb(const arc_msgs::ArcTaskGoalConstPtr &goal);

     /**
     * Callback after the explore timer gets set off
     * @param event some data about how long we explored.
     */
    void explore_timer_cb(const ros::TimerEvent &event);
};

#endif //ARC_TASKS_TASKEXPLORE_H
