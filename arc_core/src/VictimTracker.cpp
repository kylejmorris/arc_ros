#include "VictimTracker.h"
#include <string>
#include <arc_msgs/DetectedVictims.h>
#include <vector>
#include <arc_msgs/TaskRequest.h>
#include <arc_msgs/WirelessRequest.h>

using namespace std;

VictimTracker::VictimTracker(const std::string &customNamespace) {
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle(customNamespace + "victim_tracker");

    this->global = global_handle;
    this->local = local_handle;
    string ns = global_handle.getNamespace();
    ROS_INFO("ns is %s", ns.c_str());

    this->incoming_victims_sub = global_handle.subscribe(customNamespace + "detect_victim_ps/found_victims", MAX_QUEUE_SIZE, &VictimTracker::incoming_victims_cb, this);
    this->confirm_victim_task_request_pub = global_handle.advertise<arc_msgs::WirelessRequest>(customNamespace + "wifi_handler/outgoing_requests", MAX_QUEUE_SIZE, true);
    this->confirm_victim_task_response_sub = global_handle.subscribe(customNamespace + "wifi_handler/incoming_announcements", MAX_QUEUE_SIZE, &VictimTracker::incoming_confirm_victim_response_cb, this);

    tf2_ros::Buffer *victim_buffer = new tf2_ros::Buffer;
    tf2_ros::TransformListener *victim_listener = new tf2_ros::TransformListener(*victim_buffer);

    this->victim_buffer = victim_buffer;
    this->victim_listener = victim_listener;

    global_handle.param<string>("tf_prefix", tf_prefix, "");
    if(tf_prefix.size()==0) {
        ROS_ERROR("tf prefix not specified");
    }

}

void VictimTracker::start() {
    ros::Rate r(10);

    while(ros::ok()) {
        ROS_INFO("Confirmed %d victims. Potentially %d victims. Awaiting support for %d victims", this->confirmedVictims.size(), this->potentialVictims.size(), this->awaitingConfirmation.size());
        evaluatePotentialVictims();

        r.sleep();
        ros::spinOnce();
    }
}

void VictimTracker::incoming_victims_cb(const arc_msgs::DetectedVictims &msg) {
    potentialVictimsMutex.lock();

    //transform victim into map frame
    arc_msgs::DetectedVictims victims_transformed;

    geometry_msgs::TransformStamped fiducial_link_to_map_tf;
    fiducial_link_to_map_tf = this->victim_buffer->lookupTransform("map", (tf_prefix+"/base_fiducial_link"), ros::Time(0), ros::Duration(1.0));

    for(const auto &victim : msg.victims) {
        geometry_msgs::PoseStamped transformed;
        geometry_msgs::PoseStamped victim_stamped;
        victim_stamped.pose = victim.pose;
        tf2::doTransform(victim_stamped, transformed, fiducial_link_to_map_tf);

        arc_msgs::DetectedVictim transformed_victim;
        transformed_victim.victim_id = victim.victim_id;
        transformed_victim.status = victim.status;
        transformed_victim.pose = transformed.pose;

        victims_transformed.victims.push_back(transformed_victim);
    }

    for(const auto &victim : victims_transformed.victims) {
        if(!alreadyDetectedVictim(victim)) {
            this->potentialVictims.push_back(victim);
        }
    }

    potentialVictimsMutex.unlock();
}

template<typename T>
double VictimTracker::dist(const T &first, const T &second) {
    return sqrt(pow(first.x-second.x,2) + (pow(first.y-second.y,2)));
}

bool VictimTracker::alreadyDetectedVictim(const arc_msgs::DetectedVictim &victim) {
    for(const auto &other : this->awaitingConfirmation) {
        if(dist(other.pose.position, victim.pose.position)<this->MAX_DISPLACEMENT_THRESHOLD) {
            return true;
        }
    }

    for(const auto &other : this->confirmedVictims) {
        if(dist(other.pose.position, victim.pose.position)<this->MAX_DISPLACEMENT_THRESHOLD) {
            return true;
        }
    }

    for(const auto &other : this->potentialVictims) {
        if(dist(other.pose.position, victim.pose.position)<this->MAX_DISPLACEMENT_THRESHOLD) {
            return true;
        }
    }

    return false;
}

void VictimTracker::evaluatePotentialVictims() {
    static const int POTENTIAL_VICTIM_STATUS = 1;
    arc_msgs::DetectedVictims confirmVictims;

    potentialVictimsMutex.lock();
    for(auto const &victim : this->potentialVictims) {
        if(victim.status==POTENTIAL_VICTIM_STATUS) {
            //ROS_WARN("Status of victim at %f %f is  %d", victim.pose.position.x, victim.pose.position.y, victim.status);
            confirmVictims.victims.push_back(victim);
        } else {
            this->confirmedVictims.push_back(victim);
        }
    }

    //broadcast confirm victim task request.
    if(confirmVictims.victims.size()>0) {

        //move victims into pending list
        arc_msgs::DetectedVictims requestedVictims;
        for (const auto &victim : confirmVictims.victims) {
            if(!alreadyRequestedVictim(victim)) {
                this->awaitingConfirmation.push_back(victim);
                requestedVictims.victims.push_back(victim);
            }
        }

        if(requestedVictims.victims.size()>0) {
            broadcastConfirmVictimTask(requestedVictims);
        }
    }

    this->potentialVictims.clear();
    potentialVictimsMutex.unlock();
}

void VictimTracker::broadcastConfirmVictimTask(const arc_msgs::DetectedVictims &victims) {
    arc_msgs::WirelessRequest w_req;
    arc_msgs::TaskRequest req;

    //TODO [HACK]: Specify the robots actual current location as part of wireless request
    w_req.sender_location.position.x = 5.0;
    w_req.sender_location.position.y = 5.0;

    req.task_id=1;
    req.task_name = "confirm_victim";

    req.created= ros::Time::now();

    string victimList = "";

    for(int index=0; index < victims.victims.size(); index++) {
        auto &victim = victims.victims[index];
        victimList=victimList+"(0,"+to_string(victim.pose.position.x) + "," + to_string(victim.pose.position.y) +")";

        //check if there are any more victims we'll add to this list.
        if(index+1<victims.victims.size()) {
            victimList = victimList+"|";
        }
    }

    dynamic_reconfigure::StrParameter victim_list;
    victim_list.name = "victim_list";
    victim_list.value = victimList.c_str();

    req.parameters.strs.push_back(victim_list);
    w_req.task = req;

    w_req.request_type=w_req.task.TYPE_COMPLETION;

    this->confirm_victim_task_request_pub.publish(w_req);
}

void VictimTracker::incoming_confirm_victim_response_cb(const arc_msgs::WirelessAnnouncement &msg) {
    //is this announcement about victim detection?
    //ROS_INFO("Received victim announcement");
    if(msg.announcement.bools.size()>0 && msg.announcement.bools[0].name=="positive") {
        //ROS_INFO("parsing victim announcement");
        arc_msgs::DetectedVictim victim;
        victim.status = msg.announcement.bools[0].value;
        if(msg.announcement.doubles[0].name=="x") {
            victim.pose.position.x = msg.announcement.doubles[0].value;
        } else {
            victim.pose.position.x = msg.announcement.doubles[1].value;
        }

        if(msg.announcement.doubles[0].name=="y") {
            victim.pose.position.y = msg.announcement.doubles[0].value;
        } else {
            victim.pose.position.y = msg.announcement.doubles[1].value;
        }

         //TODO: Should double check here that victim isn't already tracked by us.
         this->confirmedVictimMutex.lock();
         this->confirmedVictims.push_back(victim);
         this->confirmedVictimMutex.unlock();

         ROS_INFO("Confirmed victim. Added to list.");
    }
}

bool VictimTracker::alreadyRequestedVictim(const arc_msgs::DetectedVictim &victim) {
   for(const auto &other : this->awaitingConfirmation) {
        if(dist(other.pose.position, victim.pose.position)<this->MAX_DISPLACEMENT_THRESHOLD) {
            return true;
        }
    }

    for(const auto &other : this->confirmedVictims) {
        if(dist(other.pose.position, victim.pose.position)<this->MAX_DISPLACEMENT_THRESHOLD) {
            return true;
        }
    }
    return false;
}

int main(int argc, char **argv)  {
    ros::init(argc, argv, "victim_tracker");

    VictimTracker tracker("");
    tracker.start();

    return 0;
}
