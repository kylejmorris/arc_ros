#include "ros/ros.h"
#include "../include/WifiHandler.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "wifi_handler_node");
    WifiHandler handler;
    handler.run();
    return 0;
}