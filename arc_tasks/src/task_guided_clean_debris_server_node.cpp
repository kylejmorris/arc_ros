#include "TaskGuidedCleanDebrisServer.h"

int main(int argc, char **argv) {
    ros::init(argc,argv, "task_guided_clean_debris_server");
    TaskGuidedCleanDebrisServer server;
    ros::spin();

    return 0;
}