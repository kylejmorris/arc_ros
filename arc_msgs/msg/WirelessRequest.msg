#Pertaining to information about this SPECIFIC message, not the wireless request itself.
std_msgs/Header header

#Where is the robot who sent this announcement? This is in /map coordinate frame.
geometry_msgs/Pose sender_location

#The type of Request. Default is (0), a Task Request.
uint8 request_type
uint8 TYPE_TASK=0

#the task that is being requested. This is checked if request_type=TYPE_TASK
arc_msgs/TaskRequest task
