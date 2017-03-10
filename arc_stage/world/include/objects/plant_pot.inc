define plantpot position
(
  size [.5 0.5 1]	
   block #BASE
   (
      points 8
      point[0]  [ 1  0]
      point[1]  [ 3  0]
      point[2]  [ 4  1]
      point[3]  [ 4  3]
      point[4]  [ 3  4]
      point[5]  [ 1  4]
      point[6]  [ 0  3]
      point[7]  [ 0  1]      
      z [0 0.6]
   )
   block #PLANT
   (
      points 4
      point[0]  [ 1.75 1.75 ]
      point[1]  [ 2.25 1.75 ]
      point[2]  [ 2.25 2.25 ]
      point[3]  [ 1.75 2.25 ]
      z [0.6 1]
   )
#   block #LEAF1
#   (
#      points 7
#      point[0]  [ 2.25 1.75 ]
#      point[1]  [ 2.25 2.25 ]
#      point[2]  [ 2.5  2.5  ]
#      point[3]  [ 3    2.5  ]
#      point[4]  [ 4    2    ]
#      point[5]  [ 3    1.5  ]
#      point[6]  [ 2.5  1.5  ] 
#      z [1 1.05]
#   )
   color "green"	
)
