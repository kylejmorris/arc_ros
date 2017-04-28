#include "TaskServer.h"

void TaskServer::goal_cb(const arc_msgs::ArcTaskGoalConstPtr &goal) {
    this->recent_goal.task_id = goal->task_id;

    this->startup(goal);
    this->process();
}
