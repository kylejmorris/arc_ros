#include "TaskExploreServer.h"

int main(int argc, char ** argv) {
    ros::init(argc,argv, "task_explore_server");
    TaskExploreServer server;
    ros::spin();

    return 0;
}