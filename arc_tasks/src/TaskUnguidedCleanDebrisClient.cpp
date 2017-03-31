#include "TaskUnguidedCleanDebrisClient.h"

TaskUnguidedCleanDebrisClient::TaskUnguidedCleanDebrisClient() {
    client = new ActionClient("task_explore", true);
    ROS_INFO("Waiting for task_explore action server to start.");
    client->waitForServer();
    ROS_INFO("Action server started....counting down");
}

void TaskUnguidedCleanDebrisClient::result_cb(const actionlib::SimpleClientGoalState &state,
                                  const arc_msgs::ArcTaskResultConstPtr &result) {
    ROS_INFO("Task is complete. %s",state.getText().c_str());
}
void TaskUnguidedCleanDebrisClient::doTask(const ros::TimerEvent &event) {

    arc_msgs::ArcTaskGoal goal;
    ROS_INFO("Sending task");
    client->sendGoal(goal, boost::bind(&TaskUnguidedCleanDebrisClient::result_cb, this, _1, _2));
}