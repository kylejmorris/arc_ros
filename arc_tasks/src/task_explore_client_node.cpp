#include "TaskExploreClient.h"

int main(int argc, char ** argv) {
    ros::init(argc,argv, "task_explore_client");
    TaskExploreClient client;
    ros::NodeHandle nh;
    //send a task to object after 10 seconds, to let other stuff be ready... just a hack for now
    ros::Timer timer = nh.createTimer(ros::Duration(20), &TaskExploreClient::doTask, &client, true);
    ros::spin();

    return 0;
}