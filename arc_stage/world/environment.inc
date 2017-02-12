# Tyler J. Gunn
# This include file defines elements of the environment including:
# - Walls
# - Debris
# - Victims (persons)
# - False Victims (ie false positives)

# Define a model to represent the walls in the environment.
define walls model
(
  # sombre, sensible, artistic
  color "gray40"

  # most maps will need a bounding box
  boundary 1

  gui_nose 0
  gui_grid 0
  gui_move 0
  gui_outline 0
  gripper_return 0
  fiducial_return 0
  #laser_return 1
  ranger_return 1

  # Returns a hit in the "walls" digit
  obstacle_return "0b010"

  block
  (
    points 4
	point[0] [-0.5 0.5]
	point[1] [-0.5 -0.5]
	point[2] [0.5 -0.5]
	point[3] [0.5 0.5]
	z [0.0 1.0]
  )
)

# Define a model for the debris found in the environment.
define debris model
(
  # sombre, sensible, artistic
  color "gray20"

  # most maps will need a bounding box
  #boundary 1

  gui_nose 0
  gui_grid 0
  gui_move 0
  gui_outline 0
  gripper_return 0
  fiducial_key 4
  fiducial_return 1
  #laser_return 1
  ranger_return 1

  # Returns a hit in the "debris" digit
  obstacle_return "0b100"

  # debris can be destroyed by a debris bot
  #invulnerable 0
  #hitPoints 5

  block
  (
    points 4
	point[0] [-0.5 0.5]
	point[1] [-0.5 -0.5]
	point[2] [0.5 -0.5]
	point[3] [0.5 0.5]
	z [0.0 1.0]
	#color "gray60"
  )
)


# This model defines a false positive configuration of debris in the environment.
# At a distance, it appears to the victim sensor as a potential person.  However, upon
# closer investigation, the victim sensor identifies it as a false positive.
define unperson model
(
  fiducial_key 2
  fiducial_return 2  # 2= non-person

  obstacle_return "0b100"

  color "gray40"

  size [ 0.800 1.600 0.200 ]

    # Left leg
  block (
    points 5
    point[0]  [  0.000  0.000 ]
    point[1]  [  0.000  0.100 ]
    point[2]  [ -0.200  0.100 ]
    point[3]  [ -0.500 -0.950 ]
    point[4]  [  0.000 -0.950 ]
    z [ 0.000 0.200 ]
  )

  #right leg
  block (
    points 5
    point[0]  [  0.000  0.000 ]
    point[1]  [  0.000 -0.950 ]
    point[2]  [  0.500 -0.950 ]
    point[3]  [  0.200  0.100 ]
    point[4]  [  0.000  0.100 ]
    z [ 0.000 0.200 ]
  )

  #torso
  block (
    points 4
    point[0]  [  0.200  0.100 ]
    point[1]  [  0.200  0.850 ]
    point[2]  [ -0.200  0.850 ]
    point[3]  [ -0.200  0.100 ]
    z [ 0.000 0.200 ]
  )

  # left arm
  block (
    points 5
    point[0]  [ -0.200  0.850 ]
    point[1]  [ -0.300  0.840 ]
    point[2]  [ -0.490  0.290 ]
    point[3]  [ -0.200  0.200 ]
    point[4]  [ -0.200  0.700 ]
    z [ 0.000 0.200 ]
  )

  # right arm
  block (
    points 5
    point[0]  [  0.200  0.850 ]
    point[1]  [  0.200  0.700 ]
    point[2]  [  0.200  0.200 ]
    point[3]  [  0.490  0.290 ]
    point[4]  [  0.300  0.840 ]
    z [ 0.000 0.200]
  )

  # head
  block (
    points 4
    point[0]  [ -0.150  0.850 ]
    point[1]  [  0.150  0.850 ]
    point[2]  [  0.150  1.150 ]
    point[3]  [ -0.150  1.150 ]
    z [ 0.000 0.200 ]
  )

)


# This model defines a real person.  It will be detected as a potential hit from afar, 
# and when the robot comes within range, it's victim detector will correctly identify
# it as a person.
define person model
(
  fiducial_key 2
  fiducial_return 1  # 1= person

  obstacle_return "0b100"
  ranger_return 1

  size  [ 0.800 1.600 0.2 ]

  color "red"
  
  # Left leg
  block (
    points 5
    point[0]  [  0.000  0.000 ]
    point[1]  [  0.000  0.100 ]
    point[2]  [ -0.200  0.100 ]
    point[3]  [ -0.500 -0.950 ]
    point[4]  [ -0.200 -0.950 ]
    z [ 0.000 0.200 ]
    color "bisque3"
  )

  #right leg
  block (
    points 5
    point[0]  [  0.000  0.000 ]
    point[1]  [  0.200 -0.950 ]
    point[2]  [  0.500 -0.950 ]
    point[3]  [  0.200  0.100 ]
    point[4]  [  0.000  0.100 ]
    z [ 0.000 0.200 ]
    color "bisque3"
  )

  #torso
  block (
    points 4
    point[0]  [  0.200  0.100 ]
    point[1]  [  0.200  0.850 ]
    point[2]  [ -0.200  0.850 ]
    point[3]  [ -0.200  0.100 ]
    z [ 0.000 0.200 ]
    color "DarkGreen"
  )

  # left arm
  block (
    points 5
    point[0]  [ -0.200  0.850 ]
    point[1]  [ -0.300  0.840 ]
    point[2]  [ -0.490  0.290 ]
    point[3]  [ -0.350  0.200 ]
    point[4]  [ -0.200  0.700 ]
    z [ 0.000 0.200 ]
    color "bisque3"
  )

  # right arm
  block (
    points 5
    point[0]  [  0.200  0.850 ]
    point[1]  [  0.200  0.700 ]
    point[2]  [  0.350  0.200 ]
    point[3]  [  0.490  0.290 ]
    point[4]  [  0.300  0.840 ]
    z [ 0.000 0.200]
    color "bisque3"
  )

  # head
  block (
    points 4
    point[0]  [ -0.150  0.850 ]
    point[1]  [  0.150  0.850 ]
    point[2]  [  0.150  1.150 ]
    point[3]  [ -0.150  1.150 ]
    z [ 0.000 0.200 ]
    color "bisque3"
  )
)


