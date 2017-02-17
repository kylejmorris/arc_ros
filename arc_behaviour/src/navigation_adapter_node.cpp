#include "ros/ros.h"
#include "NavigationAdapter.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation_adaptor");
    arc_behaviour::NavigationAdapter adapter;
    adapter.run();
    return 0;
}