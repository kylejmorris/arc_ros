define pioneer_base position 
(
  color "red"     # Default color.
  drive "diff"      # Differential steering model.
  gui_nose 1                    # Draw a nose on the robot so we can see which way it points
  obstacle_return 1             # Can hit things.
  #laser_return 1                 # reflects laser beams
  #ranger_return 1               # reflects sonar beams
  #blob_return 1                 # Seen by blobfinders  
  #fiducial_return 1             # Seen as "1" fiducial finders

  origin [-0.04465 0.0 0.0 0]
  #localization "gps"            
  #localization_origin [0 0 0 0]   # Start odometry at (0, 0, 0).

  # alternative odometric localization with simple error model
  # localization "odom"             # Change to "gps" to have impossibly perfect, global odometry
  #  odom_error [ 0.05 0.05 0.1 ]     # Odometry error or slip in X, Y and Theta
                                    # (Uniform random distribution)   
)

#define p3dx_sonar ranger
#(
#  # define the pose of each transducer [xpos ypos heading]
#  spose[0] [ 0.069 0.136 90 ]
#  spose[1] [ 0.114 0.119 50 ]
#  spose[2] [ 0.148 0.078 30 ]
#  spose[3] [ 0.166 0.027 10 ]
#  spose[4] [ 0.166 -0.027 -10 ]
#  spose[5] [ 0.148 -0.078 -30 ]
#  spose[6] [ 0.114 -0.119 -50 ]
#  spose[7] [ 0.069 -0.136 -90 ]
#  spose[8] [ -0.157 -0.136 -90 ]
#  spose[9] [ -0.203 -0.119 -130 ]
#  spose[10] [ -0.237 -0.078 -150 ]
#  spose[11] [ -0.255 -0.027 -170 ]
#  spose[12] [ -0.255 0.027 170 ]
#  spose[13] [ -0.237 0.078 150 ]
#  spose[14] [ -0.103 0.119 130 ]
#  spose[15] [ -0.157 0.136 90 ] 
#    
#  # define the field of view of each transducer [range_min range_max view_angle]
#  sview [0.1 5.0 30]  # min (m), max (m), field of view (deg)
#
#  # define the size of each transducer [xsize ysize] in meters
#  ssize [0.01 0.04]
#)
#
# a Pioneer 2 or 3 in standard configuration
define pioneer3dx pioneer_base
(
  # this set of blocks approximates the shape of a real Pioneer

  #  The geometry is from the Webots v5.3.0 manual. Thanks to Webots
  #  and Olivier Michel. If Stage or Gazebo do not do what you want,
  #  take a look at Webots. It's a very nice commercial simulator.
 
  # main body
  block 
  (
    points 8
    point[0] [ -0.185 -0.135 ] 
    point[1] [  0.095 -0.135 ] 
    point[2] [  0.11  -0.08  ] 
    point[3] [  0.11   0.08  ] 
    point[4] [  0.095  0.135 ] 
    point[5] [ -0.185  0.135 ]
    point[6] [ -0.215  0.1   ]
    point[7] [ -0.215 -0.1   ] 
    z [ 0.059 0.234 ]
  )

  # sonar case
  block
  (
    points 9
    point[0]  [ -0.135  0.136 ]
    point[1]  [ -0.185  0.136 ]
    point[2]  [ -0.223  0.101 ]
    point[3]  [ -0.248  0.054 ]
    point[4]  [ -0.258  0     ]
    point[5]  [ -0.248 -0.054 ]
    point[6]  [ -0.223 -0.101 ]
    point[7]  [ -0.185 -0.136 ]
    point[8]  [ -0.135 -0.136 ]
    z [ 0.184 0.234 ]
  )

  # sonar case
  block
  (
    points 9
    point[0]  [ 0.046 -0.136 ]
    point[1]  [ 0.096 -0.136 ]
    point[2]  [ 0.134 -0.101 ]
    point[3]  [ 0.159 -0.054 ]
    point[4]  [ 0.168  0     ]
    point[5]  [ 0.159  0.054 ]
    point[6]  [ 0.134  0.101 ]
    point[7]  [ 0.096  0.136 ]
    point[8]  [ 0.046  0.136 ]
    z [ 0.184 0.234 ]
  )
  
  # left wheel
  block
  (
    points 4
    point[0] [  0.083  0.177 ]
    point[1] [ -0.083  0.177 ]
    point[2] [ -0.083  0.140 ]
    point[3] [  0.083  0.140 ]
    z [0 0.165 ]
    color "gray15"
  )

  # right wheel
  block
  (
    points 4
    point[0] [  0.083  -0.14 ]
    point[1] [ -0.083  -0.14 ]
    point[2] [ -0.083  -0.177 ]
    point[3] [  0.083  -0.177 ]
    z [ 0 0.165 ]
    color "gray15"
  )

  # castor
  block
  (
    points 4
    point[3] [ -0.2475  0.012 ]
    point[2] [ -0.1825  0.012 ]
    point[1] [ -0.1825 -0.012 ]
    point[0] [ -0.2475 -0.012 ]
    z [ 0 0.065 ]
    color "gray15"
  )
 
  # lid
  block
  (
    points 22
    point[21] [  0.174 0 ]
    point[20] [  0.166 -0.056 ]
    point[19] [  0.145 -0.107 ]
    point[18] [  0.112 -0.155 ]
    point[17] [  0.064 -0.190 ]
    point[16] [  -0.074 -0.190 ]
    point[15] [  -0.096 -0.160 ]
    point[14] [  -0.151 -0.160 ]
    point[13] [  -0.2   -0.155 ]
    point[12] [  -0.236 -0.107 ]
    point[11] [  -0.256 -0.056 ]
    point[10] [  -0.264  0     ]
    point[9] [  -0.256  0.056 ]
    point[8] [ -0.236  0.107 ]
    point[7] [ -0.2    0.155 ]
    point[6] [ -0.151  0.160 ]
    point[5] [ -0.096  0.160 ]
    point[4] [ -0.074  0.190 ]
    point[3] [  0.064  0.190 ]
    point[2] [  0.112  0.155 ]
    point[1] [  0.145  0.107 ]
    point[0] [  0.166  0.056 ]
    z [ 0.234 0.24 ]

    # a dark top looks more realistic, but isn't very useful 
    # for a top-down view
    #color "gray10"
    color "red"
  )

    # The pioneer's center of rotation is offset from its center of area

  # Estimated mass in KG
  mass 23.0 
  size [0.511 0.4 0.22 ]

  #p3dx_sonar( pose [ 0 0 -0.03 0 ] ) 
)

define hokuyolaser ranger
(
  sensor(       
    range [ 0.0  5.0 ]
    fov 270
   samples 270
  )

  # generic model properties
  color "blue"
  size [ 0.07 0.07 0.05 ] # dimensions from LMS200 data sheet 
)

define hokuyorapid ranger
(
  sensor(       
    range [ 0.0  4.0 ]
    fov 240
   samples 240
  )

  color "blue"
  size [ 0.073 0.060 0.055 ]
)

