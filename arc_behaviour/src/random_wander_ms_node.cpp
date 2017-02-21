#include "ros/ros.h"
#include "RandomWanderMS.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "random_wander_ms_node");
    arc_behaviour::RandomWanderMS random_wander;
    random_wander.run();
    return 0;
}