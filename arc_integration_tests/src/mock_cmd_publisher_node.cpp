#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "mock_cmd_publisher");
    ros::NodeHandle handle;

    ros::Publisher cmd_pub = handle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Rate r(10);
    while(ros::ok()) {
        //generate message
        geometry_msgs::Twist cmd_msg;
        cmd_msg.angular.x = 1.0;
        cmd_msg.angular.y = 5.0;
        cmd_msg.angular.z= 2.0;
        cmd_msg.linear.x = 1.0;
        cmd_msg.linear.y = 9.0;
        cmd_msg.linear.z = 10.0;

        //publish message
        cmd_pub.publish(cmd_msg);
        r.sleep();
    }
    return 0;
}

