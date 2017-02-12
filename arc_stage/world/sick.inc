
# changed to ranger for ARC port
#define sicklaser ranger
#(
#  sensor(
#   laser-specific properties
#   factory settings for LMS200
#  range_max 8.0
#  fov 180.0
#  samples 361

#  samples 91 # still useful but much faster to compute
#  )
#   generic model properties
#  color "blue"
#  size [ 0.156 0.155 0.19 ] # dimensions from LMS200 data sheet
#)

define sicklaser sensor
(
	size [ 0.156 0.155 0.19 ]
	range [ 0.0 2.0 ]
	fov [ 25.0 ]	
	color_rgba [ 0.0 0.2 1.0 0.5]
	samples 360
	samples 91
)

define sicklasers ranger
(
	sicklaser( pose [ 0.085 -0.070  0.000  -50 ] )
	sicklaser( pose [ 0.100  0.000  0.000    0 ] )
	sicklaser( pose [ 0.085  0.070	0.000   50 ] )
)

# extends sicklaser to add nice-looking but relatively expensive geometry
define fancysicklaser sicklaser
(
  # bottom
  block(
    points 4
    point[0] [ -0.02 -0.077 ]
    point[1] [  0.078 -0.077 ]
    point[2] [  0.078  0.077 ]
    point[3] [ -0.02  0.077 ]
    z [0 0.02 ]
  )

  # back
  block(
    points 4
    point[0] [ -0.078 -0.077 ]
    point[1] [ -0.02  -0.077 ]
    point[2] [ -0.02   0.077 ]
    point[3] [ -0.078  0.077 ]
    z [0 0.21 ]
  )

  # top
  block( points 4
    point[0] [ -0.02 -0.077 ]
    point[1] [  0.078 -0.077 ]
    point[2] [  0.078  0.077 ]
    point[3] [ -0.02  0.077 ]
    z [0.12 0.21 ]
  )

  # laser bit
  block( points 4 point[0] [ -0.02 -0.05 ]
	 point[1] [  0.06 -0.05 ]
	 point[2] [  0.06  0.05 ]
	 point[3] [ -0.02  0.05 ]
	 z [0.02 0.12 ]
   color "gray10"
  )
)



