std_msgs/Header header

#Where is the robot who sent this announcement? This is in /map coordinate frame.
geometry_msgs/Pose sender_location

#content of the announcement. The information to broadcast.
dynamic_reconfigure/Config announcement
