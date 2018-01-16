/**
* CLASS: VictimTracker
* DATE: 04/01/18
* AUTHOR: Kyle Morris
* DESCRIPTION: Manage detected victim information in the environment.
*/

#ifndef ARC_CORE_VICTIMTRACKER_H
#define ARC_CORE_VICTIMTRACKER_H
#include <ros/ros.h>
#include <arc_msgs/DetectedVictims.h>


class VictimTracker {
private:
    ros::NodeHandle local;
    ros::NodeHandle global;

    ros::Subscriber incoming_victims_sub;

    static const int MAX_QUEUE_SIZE = 100;
public:
    VictimTracker();

    void start();

    void incoming_victims_cb(const arc_msgs::DetectedVictims &msg);
};


#endif //ARC_CORE_VICTIMTRACKER_H
