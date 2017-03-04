# Geoff Nagy
# This include file defines objects that are not sensors or robots,
# such as droppable fiducial markers, stairways, etc.

define newstairway teleporter
(
	# appearance settings
	color "blue"

	# all teleporters should have a delay to simulate the time it takes to use the stairs
	transitTime 2.0

	# any teleport-capable objects must be within this radius in order to use the teleporter
	teleportRadius 0.6

	# teleporters are always enabled
	alwayson 1

	obstacle_return 1
	ranger_return 1

	# left arm of stairway
	block
	(
		points 4
		point[0] [-1.0 1.0]
		point[1] [-1.0 0.8]
		point[2] [1.0 0.8]
		point[3] [1.0 1.0]
		z [0.0 5.0]
	)

	# right arm of stairway
	block
	(
		points 4
		point[0] [-1.0 -1.0]
		point[1] [-1.0 -0.8]
		point[2] [1.0 -0.8]
		point[3] [1.0 -1.0]
		z [0.0 5.0]
	)

	# back end of stairway
	block
	(
		points 4
		point[0] [-1.0 0.8]
		point[1] [-1.0 -0.8]
		point[2] [-0.8 -0.8]
		point[3] [-0.8 0.8]
		z [0.0 5.0]
	)

	# actual part that we can detect
	model
	(
		block
		(
			points 4
			point[0] [-0.1 0.1]
			point[1] [-0.1 -0.1]
			point[2] [0.1 -0.1]
			point[3] [0.1 0.1]
			z [0.0 0.00001]
		)

		# fiducial settings so that stairbots can detect it
		fiducial_key 4
		fiducial_return 1

		alwayson 1

		size [0.2 0.2 0.001]
	)
)

define stairway teleporter
(
	# this is really only to define the selection highlight when we drag it around
	size [ 1.2 1.2 5.0 ]

	# all teleporters should have a delay to simulate the time it takes to use the stairs
	transitTime 2.0
	
	stairway_body
	(
		# make it easy to see
		color "blue"

		# scale it to something somewhat realistic
		size [1.2 1.2 2.2]
		pose [0.0 0.0 0.0 0.0]

		# all robots can see it
		obstacle_return 1
		ranger_return 1
	)
)

# this defines the geometry of our stairway
define stairway_body model
(
	# left arm of stairway
	block
	(
		points 4
		point[0] [-0.4 0.5]
		point[1] [-0.4 -0.5]
		point[2] [-0.5 -0.5]
		point[3] [-0.5 0.5]
		z [0.0 0.8]
		color "blue"
	)

	# right arm of stairway
	block
	(
		points 4
		point[0] [0.4 0.5]
		point[1] [0.4 -0.5]
		point[2] [0.5 -0.5]
		point[3] [0.5 0.5]
		z [0.0 0.8]
		color "blue"
	)

	# back end of stairway
	block
	(
		points 4
		point[0] [0.4 -0.4]
		point[1] [0.4 -0.5]
		point[2] [-0.4 -0.5]
		point[3] [-0.4 -0.4]
		z [0.0 0.8]
		color "blue"
	)
)

# define a model for the fiducial droppables
define marker position
(
	# make them easy to see
	color "yellow"

	# this is not to scale, but it needs to be large enough to see despite its tiny size
	size [ 0.100 0.300 0.100 ]

	# very small, so robots just run right over them (we could use "0b000" instead)
	obstacle_return 0

	# too short to see with lasers
	ranger_return -1

	# invisible initially; it becomes visible when a robot deploys it
	hidden 1

	# handled by the plugin so the simulation can allocate them among the agents

	# identifies as a droppable marker
	fiducial_key 3
	fiducial_return 3
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
