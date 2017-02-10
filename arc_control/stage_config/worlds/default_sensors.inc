# Tyler J. Gunn
# 
# This include file defines the sensors common to the robots in my work.

#################################################################
# Stairway sensor
#################################################################

# This is the sensor used by the stairbots and other robots to
# detect stairways where they occur in my worlds. We use simple
# fiducial sensors, and most robots will have this kind of sensor
# in order to be able to figure out where paths to other "floors"
# may exist.
define stairway_sensor fiducial
(
	name "stairway_sensor"
	range_min 0.0
	range_max 5.0
	range_max_id 5.0
	fiducial_key 4
	ignore_zloc 1
	size [0.100 0.100 0.05]
	fov 180
)

#################################################################
# Robot sensor
#################################################################

# Robots identify one another in the environment using coded fiducials.
# We define our common "robot sensor" functionality here.
define robot_sensor fiducial
(
  name "robot_sensor"
	range_min 0.0
	range_max 6.0
	range_max_id 6.0
	fiducial_key 1
	ignore_zloc 1
	size [ 0.100 0.100 0.001 ]
    fov 180
)

#################################################################
# Victim sensor
#################################################################

# Victim detection uses fiducials.  In reality we'd use heat sensors or some other
# kinda of fancy sensors to try and determine what is a victim or not.
# The idea here is we've configured the sensor so that at a distance it is not possible
# to tell if something is a victim or not.  Once close enough, it is possible to positively
# identify a victim as a true or false victim.
define victim_sensor fiducial
(
  name "victim_sensor"
  range_min 0.0
  range_max 6.0
  range_max_id 4.0
  fiducial_key 2
  ignore_zloc 1
  size [ 0.100 0.100 0.001 ]
  fov 180
)

define debris_sensor fiducial
(
  name "debris_sensor"
  range_min 0.0
  range_max 6.0
  range_max_id 4.0
  fiducial_key 4
  ignore_zloc 1
  size [ 0.100 0.100 0.001 ]
  fov 180
)

# A very basic victim sensor.  We assume this is something relatively expendable with a
# very low accuracy.  Perhaps only an infrared sensor.  This type of sensor is low
# accuracy enough that it cannot distinguish between false positive and real victims.
# In that regard it's just a really rough guess, which would require a better robot to
# confirm.
define basic_victim_sensor victim_sensor
(
  range_min 0.0
  range_max 4.0
  range_max_id 0.00
)

#################################################################
# Fiducial marker sensor
#################################################################

# This sensor is used to detect the markers that robots will drop
# nearby detected victims

define marker_sensor fiducial
(
	name "marker_sensor"
	
	# ranger longer than victim sensor so we can see the marker first
	range_min 0.0
	range_max 8.0
	range_max_id 0.0
	fov 180
	ignore_zloc 1

	fiducial_key 3
	size [ 0.100 0.100 0.001 ]
)

#################################################################
# SmallBot sonar rangers
#################################################################

#define smallbot_sonar ranger
#(
	#scount 5

	#left left front
	#spose3[0] [  0.085 -0.070  0.000  -50 ]

 	#left front
	#spose3[1] [  0.100 -0.035  0.000  -30 ]

	#middle front
	#spose3[2] [	 0.100  0.000  0.000   0  ]

	#right front
	#spose3[3] [  0.100  0.035  0.000   30 ]

	#right right front
	#spose3[4] [  0.085  0.070	 0.000   50 ]

	#Define field of view of each transducer
	#[range_min range_max view_angle]
	#sview [0 2.0 25]

	#Define size of each transducer [xsize ysize] in meters
	#ssize [0.01 0.01]
	
	#sraycount 3
#)

define smallbot_sonar sensor
(
	size [0.01 0.05 0.01]
	range [0.0 2.0]
	fov [25]
	color_rgba [0 0 1 0.5]
	samples 3
)

define smallbot_sonars ranger
(
	smallbot_sonar( pose [ 0.085 -0.070  0.000  -50 ] )
	#smallbot_sonar( pose [ 0.100 -0.035  0.000  -30 ] )
	#smallbot_sonar( pose [ 0.100  0.000  0.000   0  ] )
	#smallbot_sonar( pose [ 0.100  0.035  0.000   30 ] )
	#smallbot_sonar( pose [ 0.085  0.070	 0.000   50 ] )
)

#################################################################
# MidBot sonar rangers
#################################################################

define midbot_sonar sensor
(
	size [0.01 0.05 0.01]
	range [0.15 6.45]
	fov [5]
	color_rgba [0 0 1 0.5]
	samples 3   #1#30#180
)

define midbot_sonars ranger
(
	midbot_sonar( pose [ -0.010  0.100  0.000  95 ] )
	#midbot_sonar( pose [  0.030  0.090  0.000  65 ] )
	#midbot_sonar( pose [  0.070  0.070  0.000  35 ] )
	#midbot_sonar( pose [  0.090  0.030  0.000  15 ] )

	# Front two by nose 
	#midbot_sonar( pose [  0.100  0.010  0.000  5 ] )
	#midbot_sonar( pose [  0.100 -0.010  0.000 -5 ] )

	# The sonars to the right of the nose.
	#midbot_sonar( pose [  0.090 -0.030  0.000 -15 ] )
	#midbot_sonar( pose [  0.070 -0.070  0.000 -35 ] )
	#midbot_sonar( pose [  0.030 -0.090  0.000 -65 ] )
	#midbot_sonar( pose [ -0.010 -0.100  0.000 -95 ] )
)

#################################################################
# Debris bot sonar rangers
#################################################################

# we don't define another sensor type, since we want to use the same
# components as the midbot sensor array (i.e., we use the sensor type
# that was defined above so we wouldn't need any new hardware, but we
# use a different physical configuration)

define debrisbot_sonars ranger
(
	# array on bottom; this covers only a narrow FOV
	
	#midbot_sonar( pose [  0.020 0.010 0.000  7 ] )
	#midbot_sonar( pose [  0.040 0.010 0.000  21 ] )
	#midbot_sonar( pose [  0.060 0.010 0.000  28 ] )

	#midbot_sonar( pose [ -0.020 -0.010 0.000 -7 ] )
	#midbot_sonar( pose [ -0.040 -0.010 0.000 -21 ] )
	#midbot_sonar( pose [ -0.060 -0.010 0.000 -28 ] )

	# array on top covers a wider FOV

	#midbot_sonar( pose [  0.020 0.010 0.200  10 ] )
	#midbot_sonar( pose [  0.040 0.010 0.200  30 ] )
	#midbot_sonar( pose [  0.060 0.010 0.200  50 ] )
	#midbot_sonar( pose [  0.080 0.010 0.200  70 ] )
	#midbot_sonar( pose [  0.080 0.010 0.200  90 ] )

	midbot_sonar( pose [ -0.020 -0.010 0.200 -10 ] )
	#midbot_sonar( pose [ -0.040 -0.010 0.200 -30 ] )
	#midbot_sonar( pose [ -0.060 -0.010 0.200 -50 ] )
	#midbot_sonar( pose [ -0.080 -0.010 0.200 -70 ] )
	#midbot_sonar( pose [ -0.080 -0.010 0.200 -90 ] )
)

# Use sonars such as the ones at:
# http://www.robotshop.ca/maxbotix-ez3-ultrasonic-ranger-1.html
# These sonars are available for $27 in quantity, and have spec:
# - narrow beam
# - 0.15m minimum range
# - 6.45m maximum range
# - 1" resolution (small enough for our purposes)
define midbot_sonar_old ranger
(
  scount 10
  # The sonars to the left of the nose.
  #spose3[0] [ -0.010  0.100  0.000  95 ]
  #spose3[1] [  0.010  0.095  0.000  80 ]
  #spose3[1] [  0.030  0.090  0.000  65 ]
  #spose3[3] [  0.050  0.085  0.000  50 ]
  #spose3[2] [  0.070  0.070  0.000  35 ]
  #spose3[5] [  0.080  0.050  0.000  25 ]
  #spose3[3] [  0.090  0.030  0.000  15 ]

  # Front two by nose 
  #spose3[4] [  0.100  0.010  0.000  5 ]
  #spose3[5] [  0.100 -0.010  0.000 -5 ]

  # The sonars to the right of the nose.
  #spose3[6] [  0.090 -0.030  0.000 -15 ]
  #spose3[10] [  0.080 -0.050  0.000 -25 ]
  #spose3[7] [  0.070 -0.070  0.000 -35 ]
  #spose3[12] [  0.050 -0.085  0.000 -50 ]
  #spose3[8] [  0.030 -0.090  0.000 -65 ]
  #spose3[14] [  0.010 -0.095  0.000 -80 ]
  #spose3[9] [ -0.010 -0.100  0.000 -95 ]

  #Define field of view of each transducer
	#[range_min range_max view_angle]
	sview [0.15 6.45 5]

	#Define size of each transducer [xsize ysize] in meters
	ssize [0.01 0.01]
	
  # How many rays will be traced by each ranger.
  sraycount 3
)


