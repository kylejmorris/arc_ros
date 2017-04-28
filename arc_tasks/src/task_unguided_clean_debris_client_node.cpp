#include <ros/ros.h>
#include "TaskUnguidedCleanDebrisClient.h"

int main(int argc, char ** argv) {
    ros::init(argc,argv, "task_unguided_clean_debris_client");
    TaskUnguidedCleanDebrisClient client;
    ros::NodeHandle nh;

    //send a task to object after 10 seconds, to let other stuff be ready... just a hack for now
    ros::Timer timer = nh.createTimer(ros::Duration(10), &TaskUnguidedCleanDebrisClient::doTask, &client, true);
    ros::spin();

    return 0;
}