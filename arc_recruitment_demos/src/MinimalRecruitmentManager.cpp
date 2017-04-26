#include "MinimalRecruitmentManager.h"
#include <ros/ros.h>

#define DEFAULT_RATE 10
#define MAX_QUEUE_SIZE 1000

MinimalRecruitmentManager::MinimalRecruitmentManager() : local_handle("recruitment_manager") {
    this->task_requests_sub = this->global_handle.subscribe("communication_manager/task_requests", MAX_QUEUE_SIZE, &MinimalRecruitmentManager::task_request_cb, this);
    this->task_request_pub = this->global_handle.advertise<arc_msgs::TaskRequest>("task_handler/task_requests", MAX_QUEUE_SIZE);
}

void MinimalRecruitmentManager::process() {
    ros::Rate rate(DEFAULT_RATE);

    while(ros::ok()) {
        ros::spinOnce();
    }
}

void MinimalRecruitmentManager::task_request_cb(const arc_msgs::TaskRequest &req) {
    ROS_INFO("Received task request. Sending it over to the task_handler!");
    this->task_request_pub.publish(req);
}
