#include "TaskServer.h"

void TaskServer::goal_cb(const arc_msgs::ArcTaskGoalConstPtr &goal) {
    this->startup(goal);
    this->process();
}
