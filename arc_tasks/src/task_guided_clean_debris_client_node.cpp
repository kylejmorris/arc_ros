#include <ros/ros.h>
#include "TaskGuidedCleanDebrisClient.h"

int main(int argc, char ** argv) {
    ros::init(argc,argv, "task_guided_clean_debris_client");
    TaskGuidedCleanDebrisClient client;
    ros::NodeHandle nh;

    //send a task to object after 10 seconds, to let other stuff be ready... just a hack for now
    ros::Timer timer = nh.createTimer(ros::Duration(10), &TaskGuidedCleanDebrisClient::doTask, &client, true);
    ros::spin();

    return 0;
}