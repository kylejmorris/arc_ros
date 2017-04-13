#include <XmlRpcException.h>
#include "arc_msgs/TaskRequest.h"
#include "../include/CommunicationManager.h"

#define MAX_QUEUE_SIZE 1000
#define DEFAULT_ROS_RATE 10

using namespace XmlRpc;

CommunicationManager::CommunicationManager() {
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle("communication_manager");
    this->global_handle = global_handle;
    this->local_handle = local_handle;

    //reading parameters
    ROS_INFO("Setting up Communication manager");
    //setting up subscribers
    this->incoming_announcements_sub = global_handle.subscribe("wifi_handler/incoming_announcements", MAX_QUEUE_SIZE, &CommunicationManager::process_incoming_announcements_cb, this);
    this->incoming_requests_sub = global_handle.subscribe("wifi_handler/incoming_requests", MAX_QUEUE_SIZE, &CommunicationManager::process_incoming_requests_cb, this);
    this->incoming_responses_sub = global_handle.subscribe("wifi_handler/incoming_responses", MAX_QUEUE_SIZE, &CommunicationManager::process_incoming_responses_cb, this);

    //setting up publishers
    this->outgoing_announcements_pub = global_handle.advertise<arc_msgs::WirelessAnnouncement>("wifi_handler/outgoing_announcements", MAX_QUEUE_SIZE);
    this->outgoing_requests_pub = global_handle.advertise<arc_msgs::WirelessRequest>("wifi_handler/outgoing_requests", MAX_QUEUE_SIZE);
    this->outgoing_responses_pub = global_handle.advertise<arc_msgs::WirelessResponse>("wifi_handler/outgoing_responses", MAX_QUEUE_SIZE);

    this->task_requests_pub = local_handle.advertise<arc_msgs::TaskRequest>("task_requests", MAX_QUEUE_SIZE);

    /**
     * Populating valid task list
     */
     try {
         XmlRpcValue task_list;
         this->global_handle.getParam("task_handler/valid_tasks", task_list);

         ROS_INFO("found %d tasks ", task_list.size());
         for (unsigned i = 0; i < task_list.size(); i++) {
             if (task_list[i].getType() == XmlRpcValue::TypeString) {
                 std::string content = task_list[i];
                 ROS_INFO("Found a task in task_list: %s", content.c_str());
                 this->valid_tasks.push_back(content);
             }
         }
     } catch(XmlRpcException problem) {
         ROS_WARN("error in setting up Communication Manager. Make sure valid task list is specified on parameter server. [%s]", problem.getMessage().c_str());
     }
}

void CommunicationManager::run() {
    ros::Rate rate(DEFAULT_ROS_RATE);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

void CommunicationManager::process_incoming_announcements_cb(arc_msgs::WirelessAnnouncement announcement) {
    ROS_DEBUG("Processing incoming wireless announcement");
}

bool CommunicationManager::isTaskRequestValid(arc_msgs::TaskRequest msg) {
    bool result = true;

    if(msg.task_id<=0) {
        result = false;
        ROS_WARN("Task request must have ID > 0. Task request %d ignored.", msg.task_id);
    }

    if(msg.created.sec<=0) {
        result = false;
        ROS_WARN("Task creation time %d is invalid. Must be > 0", msg.created.sec);
    }

    if(msg.request_type==msg.TYPE_COMPLETION) {
        if(std::find(this->valid_tasks.begin(), valid_tasks.end(), msg.task_name)==valid_tasks.end()) {
            ROS_WARN("Task name %s is not a valid task. Task request %d ignored.", msg.task_name.c_str());
            result = false;
        }
    }

    return result;
}

void CommunicationManager::process_incoming_requests_cb(arc_msgs::WirelessRequest request) {
    ROS_DEBUG("Processing incoming wireless request");

    if(this->isTaskRequestValid(request.task)) {
        arc_msgs::TaskRequest task = request.task;
        this->task_requests_pub.publish(task);
    }
}

void CommunicationManager::process_incoming_responses_cb(arc_msgs::WirelessResponse response) {
    ROS_DEBUG("Processing incoming wireless response");
}
