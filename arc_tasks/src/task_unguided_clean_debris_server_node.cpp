#include "TaskUnguidedCleanDebrisServer.h"

int main(int argc, char **argv) {
    ros::init(argc,argv, "task_unguided_clean_debris_server");
    TaskUnguidedCleanDebrisServer server;
    ros::spin();

    return 0;
}