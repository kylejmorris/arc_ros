#include "TaskConfirmVictimServer.h"

int main(int argc, char ** argv) {
    ros::init(argc,argv, "task_confirm_victim_server");
    TaskConfirmVictimServer server;

    ros::spin();

    return 0;
}