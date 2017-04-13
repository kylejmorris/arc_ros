#include "TaskGuidedCleanDebrisClient.h"
#include "dynamic_reconfigure/Config.h"

TaskGuidedCleanDebrisClient::TaskGuidedCleanDebrisClient(){
    client = new ActionClient("task_guided_clean_debris", true);
    ROS_INFO("Waiting for task_explore action server to start.");
    client->waitForServer();
    ROS_INFO("Action server started....counting down");
}

void TaskGuidedCleanDebrisClient::result_cb(const actionlib::SimpleClientGoalState &state,
                                  const arc_msgs::ArcTaskResultConstPtr &result) {
    ROS_INFO("Task is complete. %s",state.getText().c_str());
}

void TaskGuidedCleanDebrisClient::doTask(const ros::TimerEvent &event) {
    arc_msgs::ArcTaskGoal goal;
    //Some contrived data. These debris exist in the environment.
    dynamic_reconfigure::StrParameter debris_list_param;
    debris_list_param.name = "debris_list";
    debris_list_param.value = "(1,9.06,6.43)|(6,7.93,14.74)|(4,14.74,15.65)";
    goal.parameters.strs.push_back(debris_list_param);

    ROS_INFO("Sending task");
    client->sendGoal(goal, boost::bind(&TaskGuidedCleanDebrisClient::result_cb, this, _1, _2));
}