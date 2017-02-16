#This is the detection info of a victim in arc. There is no transform info/ie we assume this detection takes place from a current robots perspective (ie base_link)
int32 victim_id		#id of the robot
int32 status		#postiive, negative, unkown, or uncertain
geometry_msgs/Pose pose	#Position of the detected robot.
