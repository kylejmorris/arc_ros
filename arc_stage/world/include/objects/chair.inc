define chair position
(
  size [0.4 0.4 0.8]	
   block #TABLE
   (
      points 4
      point[0]  [ 0   0]
      point[1]  [ 0.4 0]
      point[2]  [ 0.4 0.4]
      point[3]  [ 0   0.4]
      z [0.4 0.45]
   )
   block #LEG1
   (
      points 4
      point[0]  [ 0    0]
      point[1]  [ 0.05 0]
      point[2]  [ 0.05 0.05]
      point[3]  [ 0    0.05]
      z [0 0.4]
   )
   block #LEG2
   (
      points 4
      point[0]  [ 0.35 0]
      point[1]  [ 0.4  0]
      point[2]  [ 0.4  0.05]
      point[3]  [ 0.35 0.05]
      z [0 0.4]
   )
   block #LEG3
   (
      points 4
      point[0]  [ 0.35 0.35]
      point[1]  [ 0.4  0.35]
      point[2]  [ 0.4  0.4]
      point[3]  [ 0.35 0.4]
      z [0 0.4]
   )
   block #LEG4
   (
      points 4
      point[0]  [ 0    0.35]
      point[1]  [ 0.05 0.35]
      point[2]  [ 0.05 0.4]
      point[3]  [ 0    0.4]
      z [0 0.4]
   )

   block #BACK
   (
      points 4
      point[0]  [0   0.35]
      point[1]  [0.4 0.35]
      point[2]  [0.4 0.4]
      point[3]  [0   0.4]
      z [0.45 .8]
   )

   color "brown"	
)
