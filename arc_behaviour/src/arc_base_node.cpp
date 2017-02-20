#include "ros/ros.h"
#include "../include/ArcBase.h"
#define PUBLISH_RATE 10 //measured in hz
#define MAX_PUBLISH_QUEUE_SIZE 1000

int main(int argc, char **argv) {
    ros::init(argc, argv, "arc_base");

    ros::NodeHandle nh("arc_base");

    arc_behaviour::ArcBase arc_base(&nh);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", MAX_PUBLISH_QUEUE_SIZE);

    //activating schemas
    arc_base.setup();

    //run main loop
    ros::Rate r(PUBLISH_RATE);
    while(ros::ok) {
        //geometry_msgs::Twist action_vec = arc_base.getActionVector();
        //cmd_vel_pub.publish(action_vec);
        //TODO: Don't use cmd_vel directly from object. Have to obtain actionvector via callbacks.
        r.sleep();
    }

    return 0;
}
