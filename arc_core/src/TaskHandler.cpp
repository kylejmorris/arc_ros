#include "../include/TaskHandler.h"
#include <XmlRpcException.h>

using namespace XmlRpc;

#define MAX_QUEUE_SIZE 1000
#define DEFAULT_RATE 10

TaskHandler::TaskHandler() : local_handle("task_handler") {
    this->task_request_sub = this->local_handle.subscribe("task_requests", MAX_QUEUE_SIZE, &TaskHandler::task_handler_goal_cb, this);

    startupActionClients();
}

bool TaskHandler::isAcceptableRequest(const TaskHandler::TaskGoal &goal) {
    //check if task with this name exists
    if(this->task_clients[goal.task_name]==NULL) {
        ROS_WARN("Task request has invalid name: %s. Ignored request.", goal.task_name.c_str());
        return false;
    }
    return true;
}

void TaskHandler::startupActionClients() {
    ROS_INFO("Setting up action clients.");
    XmlRpcValue task_names;

    try {
        ros::param::get("task_handler/valid_tasks", task_names);

        for(unsigned i=0; i< task_names.size(); i++) {
            if(task_names[i].getType()==XmlRpcValue::TypeString) {
                std::string content = task_names[i];
                ROS_INFO("Found task named %s", content.c_str());
                std::string task_name = "task_" + content;
                ActionClient *client = new ActionClient(task_name, true);
                this->task_clients.insert({content, client});
            } else {
                ROS_WARN("TaskHandler::startupActionClients(): Found non-string value in task_list. Value ignored.");
            }
        }
    } catch (XmlRpcException &ex) {
        ROS_WARN("Problem setting up ActionClients for Task_handler %s", ex.getMessage().c_str());
    }
}

void TaskHandler::processRequest(const TaskHandler::TaskGoal &goal) {
    //TODO: Absolutely must unit test this aspect of the framework before adding more complexity on top of it.
    if(!isAcceptableRequest(goal)) {
        rejectRequest(goal);
    } else {
        //call upon the task if it is acceptable
        acceptRequest(goal);
    }
}

void TaskHandler::acceptRequest(const TaskHandler::TaskGoal goal) {
    ROS_ASSERT(isAcceptableRequest(goal));
    //TODO test: If task is accepted, test to ensure it is in backlog immediatly after
    arc_msgs::ArcTaskGoal action_goal; //breaking down project into action
    action_goal.parameters = goal.parameters;
    ROS_INFO("Accepted goal for task %s", goal.task_name.c_str());
    this->task_clients.at(goal.task_name)->sendGoal(action_goal, boost::bind(&TaskHandler::task_handler_result_cb, this, _1, _2));
}

void TaskHandler::rejectRequest(const TaskHandler::TaskGoal goal) {
    ROS_INFO("Request rejected");
}

void TaskHandler::process() {
    ros::Rate rate(DEFAULT_RATE);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

void TaskHandler::task_handler_goal_cb(const TaskHandler::TaskGoal &goal) {
    ROS_INFO("task_handler received a task request.");
    this->processRequest(goal);
}

void TaskHandler::task_handler_result_cb(const actionlib::SimpleClientGoalState &state,
                                         const arc_msgs::ArcTaskResultConstPtr &result) {
    ROS_INFO("RESULT Callback called.");
}
