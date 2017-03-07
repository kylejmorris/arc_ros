include "arc_sensors.inc"
include "pioneer3.inc"
include "sick.inc"

# ARC ROS robots by Kyle Morris:
#    -based on work by Tyler J. Gunn
#    -changes made to support Stage 4 made by Geoff Nagy, as well as
# 	the addition of the stairbot and debrisbot
# This include file defines robots used in arc ros


# stairbot
define stairbot_body position
(
	block
	(
		points 4
		point[0] [-0.4 0.4]
		point[1] [-0.4 -0.4]
		point[2] [0.4 -0.4]
		point[3] [0.4 0.4]
		z [0.0 0.2]
	)

	gui_noise 1
	mass 20.0	# arbitrary (heavier than smallbot)
	drive "diff"
	localization "gps"
	odom_error [0.0 0.0 0.0 0.0]

	usesTeleporters 1
)

define stairbot stairbot_body
(
	size [0.4 0.4 0.2]
	color "green"
	
	# basic obstacle avoidance sensors
	#smallbot_sonar
	#(
	#	pose [0.0 0.0 -0.1 0.0]
	#	alwayson 1
	#)

	#midbot_sonars
	#(
	#	alwayson 1
	#	pose [0 0 -0.1 0]
	#)

	# must be able to detect other robots
  	robot_sensor
	(
		alwayson 1
	)

	# must be able to detect stairways
	stairway_sensor
	(
		alwayson 1
	)

	# must use teleporters in order to climb "stairs"
	usesTeleporters 1

	# stairbots don't get victim sensors and thus don't need fiducial sensors, either

	# identifies as a robot
  	fiducial_key 1
 	fiducial_key 1
	obstacle_mask "0b111"		# collides with everything
	obstacle_return "0b001"		# an "everything else" obstacle type
)

#################################################################
### DEBRISBOT
#################################################################

# define the body of a debris-removing robot
define debrisbot_body position
(
	omniscient
	(   
		alwayson 1
		pose [ 0.0 0.0 0.0 0.0 ]
	)

	# main body
	block
	(
		points 4
		point[0] [-0.2 0.3]
		point[1] [-0.2 -0.3]
		point[2] [0.2 -0.3]
		point[3] [0.2 0.3]
		z [0.0 0.15]
		color "DarkSlateBlue"
	)

	# treads on sides
	block
	(
		
		points 4
		point[0] [-0.15 0.3]
		point[1] [-0.15 0.35]
		point[2] [0.15 0.35]
		point[3] [0.15 0.3]
		z [0.0 0.06]
		color "gray20"
	)
	block
	(
		points 4
		point[0] [-0.15 -0.3]
		point[1] [-0.15 -0.35]
		point[2] [0.15 -0.35]
		point[3] [0.15 -0.3]
		z [0.0 0.06]
		color "gray20"
	)

	# neck
	block
	(
		points 4
		point[0] [-0.06 0.02]
		point[1] [-0.06 -0.02]
		point[2] [-0.02 -0.02]
		point[3] [-0.02 0.02]
		z [0.15 0.2]
		color "gray20"
	)

	# head
	block
	(
		points 4
		point[0] [-0.07 0.12]
		point[1] [-0.07 -0.12]
		point[2] [0.04 -0.12]
		point[3] [0.04 0.12]
		z [0.2 0.24]
		color "gray10"
	)

	# eyes
	block
	(
		points 4
		point[0] [0.04 0.09]
		point[1] [0.04 0.06]
		point[2] [0.05 0.06]
		point[3] [0.05 0.09]
		z [0.21 0.23]
		color "red"
	)
	block
	(
		points 4
		point[0] [0.04 -0.09]
		point[1] [0.04 -0.06]
		point[2] [0.05 -0.06]
		point[3] [0.05 -0.09]
		z [0.21 0.23]
		color "red"
	)

	# removal equipment on front
	block
	(
		points 4
		point[0] [0.2 0.3]
		point[1] [0.25 0.3]
		point[2] [0.25 0.2]
		point[3] [0.2 0.2]
		z [0.0 0.05]
		color "orange4"
	)
	block
	(
		points 4
		point[0] [0.2 0.07]
		point[1] [0.25 0.07]
		point[2] [0.25 -0.07]
		point[3] [0.2 -0.07]
		z [0.0 0.065]
		color "orange4"
	)
	block
	(
		points 4
		point[0] [0.2 -0.2]
		point[1] [0.25 -0.2]
		point[2] [0.25 -0.3]
		point[3] [0.2 -0.3]
		z [0.0 0.05]
		color "orange4"
	)

	gui_nose 1
	mass 40.00
	localization "gps"
	odom_error [ 0.0 0.0 0.0 0.0 ]
)

# define a debris robot with sonar capabilities
define debrisbot debrisbot_body
(
	size [0.4 0.5 0.4]				# large-ish robot (large enough to clear debris)
	omniscient
	(   
		alwayson 1
		pose [ 0.0 0.0 0.0 0.0 ]
	)

	debris_sensor
	(
		alwayson 1
		pose [0 0 0.5 0]
	)

	# can detect potential victims only
	basic_victim_sensor
	(
		alwayson 1
		pose [0 0 0.45 0]			# mounted on top of robot
	)

	# can detect other robots
 	robot_sensor
	(
		alwayson 1
		pose [0 0 0.5 0]
	)

	# can detect droppable fiducial markers
	marker_sensor
	(
		alwayson 1
	)

	fiducial_key 1	#identifies as robot
	fiducial_return 1
	obstacle_mask "0b111"			# collides with everything
	obstacle_return "0b001"			# is an obstacle in the "everything else" category
	drive "omni"	

	# the debrisbot uses a tracked drive, meaning that it should be able to overcome short obstacles
	obstacle_mask "0b011"
)

#################################################################
### SMALLBOT (minbot)
#################################################################

#Define the body of a small robot.
define minbot_body position
(
	block (
		points 6
    point[0] [  0.100  0.050 ]
    point[1] [  0.060  0.100 ]
    point[2] [ -0.100  0.100 ]
    point[3] [ -0.100 -0.100 ]
    point[4] [  0.060 -0.100 ]
    point[5] [  0.100 -0.050 ]
		z [0 0.10] 
	)

	omniscient
	(   
		alwayson 1
		pose [ 0.0 0.0 0.0 0.0 ]
	)
  	
	robot_sensor
	(
		alwayson 1
	)
  
	basic_victim_sensor
	(
		alwayson 1
 	)

	marker_sensor
	(
		alwayson 1
 	)

	debris_sensor
	(
		alwayson 1
 	)


	gui_nose 1
	mass 5.0
	drive "omni"
#localization_origin [0 0 0 0]   # Start odometry at (0, 0, 0).
#  localization "odom"
localization "gps"

  odom_error [ 0.0 0.0 0.0 0.0 ]

)

#Define a small robot with sonar capabilities
define minbot minbot_body
(
	size [0.100 0.100 0.080]
	color "purple"

	basic_victim_sensor (
    		alwayson 1
  	)

  fiducial_key 1
 
	#collides with everything
	obstacle_mask "0b111"
	#is an obstacle in the "everything else" category
	obstacle_return "0b001"
	drive "omni"

)

define smallbot_w_robot_sensor smallbot
(
  robot_sensor (
		alwayson 1
	)
)


#################################################################
# MidBot
#################################################################

define midbot_body position
(
  # the main body of the robot.
  block (
    points 7
    
    # Start at the "nose" of the robot
    point[0] [  0.105  0.000 ]
    point[1] [  0.065  0.085 ]
    point[2] [ -0.022  0.105 ]
    point[3] [ -0.095  0.045 ]
    point[4] [ -0.095 -0.045 ]
    point[5] [ -0.022 -0.105 ]
    point[6] [  0.065 -0.085 ]
    z [ 0.000 0.100 ]

    color "DarkBlue" 
  )

  # some nice looking top cap
  block (
    points 3
    point[0] [  0.083  0.000 ]
    point[1] [  0.043  0.070 ]
    point[2] [  0.043 -0.070 ]
    z [ 0.100 0.150 ]

    color "MediumPurple3" 
  )

  # the back part of the fancy-pants top cap
  block (
    points 4
    point[0] [  0.043  0.070 ]
    point[1] [ -0.055  0.060 ]
    point[2] [ -0.055 -0.060 ]
    point[3] [  0.043 -0.070 ]
    z [ 0.100 0.150 ]

    color "MediumPurple3"
  )

  gui_nose 1
  mass 10.00
  drive "omni" #relaxed assumption for now. Should be diff.
#localization_origin [0 0 0 0]   # Start odometry at (0, 0, 0).
# localization "odom"
  localization "gps"

  odom_error [ 0.0 0.0 0.0 0.0 ]
)

define midbot midbot_body
(
 	size [ 0.200 0.200 0.150 ]
  
	omniscient
	(   
		alwayson 1
		pose [ 0.0 0.0 0.0 0.0 ]
	)

 	robot_sensor
	(
		alwayson 1
	)
  
 	victim_sensor
	(
		alwayson 1
	)

	marker_sensor
	(
		alwayson 1
	)

	# Fiducial key = 1 so that this robot can be detected by the robot detector
	fiducial_key 1

	obstacle_mask "0b111"
	obstacle_return "0b001"
)


#################################################################
# MaxBot
#################################################################

# This model represents a typical wheeled robot that cannot navigate in debris:
# Notice that it starts out in the open and will become stuck on the first debris
# it encounters.
define maxbot pioneer3dx
(		  
	omniscient
	(   
		alwayson 1
		pose [ 0.0 0.0 0.0 0.0 ]
	)

  	
	robot_sensor
	(
		alwayson 1
	)
  
	basic_victim_sensor
	(
		alwayson 1
 	)

	marker_sensor
	(
		alwayson 1
 	)

	debris_sensor
	(
		alwayson 1
 	)

	# Return hit in the "everything else" category
	obstacle_return "0b001"
	color "red"
	# the maxbot should use a tracked drive, meaning that it should be able to overcome short obstacles
	obstacle_mask "0b011"

	# Fiducial key = 1 so that this robot can be detected by the robot detector
	fiducial_key 1
	fiducial_return 1
	#localization_origin [0 0 0 0]   # Start odometry at (0, 0, 0).
	#localization "odom"
	localization "gps"
 	#odom_error [ 0.0 0.0 0.0 0.0 ]
)

#define wheeled_maxbot maxbot
#(
  # The obstacle mask is set so that the robot will collide with
  # walls, debris, and everything else.
  #obstacle_mask "0b111"
#)

#define tracked_maxbot maxbot
#(
  # The obstacle mask is set so that the robot will collide with
  # walls and everything else.
  #obstacle_mask "0b011"
#)
