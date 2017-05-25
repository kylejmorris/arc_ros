#include "TaskExploreServer.h"
#include "arc_msgs/ToggleSchema.h"

TaskExploreServer::TaskExploreServer() : server(global_handle, "task_explore", boost::bind(&TaskExploreServer::goal_cb, this, _1), false)
{
    ros::NodeHandle local_handle("task_explore_server");
    ros::Timer timer = global_handle.createTimer(ros::Duration(60), &TaskExploreServer::explore_timer_cb, this, false);
    timer.stop(); //make sure this thing isn't running

    this->local_handle = local_handle;
    this->explore_timer = timer;
    //TODO: Handle pre-empt callback as well

    this->arc_base_client = global_handle.serviceClient<arc_msgs::ToggleSchema>("arc_base/toggle_schema");
    this->server.start();
}

void TaskExploreServer::goal_cb(const arc_msgs::ArcTaskGoalConstPtr &goal) {
    ROS_INFO("Executing task_explore");

    this->startup(goal);
    this->process();
}

void TaskExploreServer::explore_timer_cb(const ros::TimerEvent &event) {
    //State transition
    ROS_INFO("Explore timer cb called. Started at %d and ended at %d",event.last_real, event.current_real);
    this->state = STATE_DoneExploring;
}

void TaskExploreServer::shutdown() {
    arc_msgs::ToggleSchema request;

    dynamic_reconfigure::BoolParameter random_wander_ms;
    random_wander_ms.name = "random_wander_ms";
    random_wander_ms.value = false;
    request.request.schema.push_back(random_wander_ms); //allow robot to wander around randomly

    this->arc_base_client.call(request);
    this->result.task_id = this->recent_goal.task_id;

    this->result.completed = true; //always marked as completed, since this is just a timed task.
    this->result.final_state = "DoneCleanDebris";
}

void TaskExploreServer::startup(const arc_msgs::ArcTaskGoalConstPtr &goal) {
    this->state = STATE_StartExploring;

    ROS_INFO("Starting up task_explore");
}

void TaskExploreServer::StateStartExploring() {
    arc_msgs::ToggleSchema request;

    dynamic_reconfigure::BoolParameter random_wander_ms;
    random_wander_ms.name = "random_wander_ms";
    random_wander_ms.value = true;
    request.request.schema.push_back(random_wander_ms); //allow robot to wander around randomly

    this->arc_base_client.call(request);

    //start the timer
    this->explore_timer.start();

    this->state = STATE_Exploring;
}

void TaskExploreServer::StateExploring() {
}

void TaskExploreServer::StateDoneExploring() {
    //MARK TASK AS SUCCESSFUL HERE. WE ARE DONE
    result.completed = true;
    result.final_state = "STATE_DoneExploring";
    this->explore_timer.stop();
    this->shutdown();
}

void TaskExploreServer::process() {
    ros::Rate r(10);

    //toggling of server (setSucceeded() etc, should only be done in this loop, not within state methods)
    while(ros::ok() && server.isActive()) {
        if (state == STATE_StartExploring) {
            ROS_INFO("In state: StartExploring");
            StateStartExploring();
        } else if (state == STATE_Exploring) {
            StateExploring();
        } else if (state == STATE_DoneExploring) {
            ROS_INFO("In state: DoneExploring");
            StateDoneExploring();
            server.setSucceeded(this->result);
        }

        r.sleep();
    }
}
