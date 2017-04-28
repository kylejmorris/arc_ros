#include "ros/ros.h"
#include "MoveToGoalMS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_to_goal_ms_node");
    arc_behaviour::MoveToGoalMS move_to_goal;
    move_to_goal.run();
    return 0;
}