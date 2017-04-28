#include "../include/TaskHandler.h"

int main(int argc, char ** argv) {
    ros::init(argc,argv, "task_handler");

    TaskHandler handler;
    handler.process();

    return 0;
}