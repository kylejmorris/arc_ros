#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "mock_sensor_publisher");
    ros::NodeHandle handle;

    ros::Publisher odom_pub = handle.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Publisher scan_pub = handle.advertise<sensor_msgs::LaserScan>("scan", 1000);

    ros::Rate r(10);

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    unsigned int num_readings = 100;
    double laser_frequency = 40;
    double ranges[num_readings];
    double intensities[num_readings];

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    int count = 0; //for laser scan

    while(handle.ok()){

        ros::spinOnce();               // check for incoming messages

        //handle odometry
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;

//generate some fake data for our laser scan
        for(unsigned int i = 0; i < num_readings; ++i){
            ranges[i] = count;
            intensities[i] = 100 + count;
        }
        ros::Time scan_time = ros::Time::now();

        //populate the LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan_time;
        scan.header.frame_id = "laser_frame";
        scan.angle_min = -1.57;
        scan.angle_max = 1.57;
        scan.angle_increment = 3.14 / num_readings;
        scan.time_increment = (1 / laser_frequency) / (num_readings);
        scan.range_min = 0.0;
        scan.range_max = 100.0;

        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);
        for(unsigned int i = 0; i < num_readings; ++i){
            scan.ranges[i] = ranges[i];
            scan.intensities[i] = intensities[i];
        }

        scan_pub.publish(scan);
        ++count;
        r.sleep();
    }
}