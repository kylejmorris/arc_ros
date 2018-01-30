#include "TaskConfirmVictimClient.h"
#include "dynamic_reconfigure/Config.h"

TaskConfirmVictimClient::TaskConfirmVictimClient(){
    client = new ActionClient("task_confirm_victim", true);
    ROS_INFO("Waiting for task_confirm_victim action server to start.");
    client->waitForServer();
    ROS_INFO("Action server started....counting down");
}

void TaskConfirmVictimClient::result_cb(const actionlib::SimpleClientGoalState &state,
                                            const arc_msgs::ArcTaskResultConstPtr &result) {
    ROS_INFO("Task is complete. %s",state.getText().c_str());
}

void TaskConfirmVictimClient::doTask(const ros::TimerEvent &event) {
    arc_msgs::ArcTaskGoal goal;
    //Some contrived data. These debris exist in the environment.
    dynamic_reconfigure::StrParameter debris_list_param;
    debris_list_param.name = "victim_list";
    debris_list_param.value = "(2,12.536,4.015)|(3,5.45,7.913)|(1,9.0,6.43)";
    goal.parameters.strs.push_back(debris_list_param);

    ROS_INFO("Sending task");
    client->sendGoal(goal, boost::bind(&TaskConfirmVictimClient::result_cb, this, _1, _2));
}