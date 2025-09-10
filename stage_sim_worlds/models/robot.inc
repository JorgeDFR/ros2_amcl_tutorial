include "models/hokuyo_utm30lx.inc"

define robot position (
  drive "diff"            # "diff", "omni", "car" (v + steer angle)
  localization "odom"     # "gps" (perfect accuracy), "odom" (odometry model)
  odom_error [ 0.025 0.05 0.01 0.02 0.02 0.04 ]
      # [x xstd y ystd theta thetastd]
      # gaussian error model with a mean and standard deviation
      # nonzero mean introduces a bias into the odometry
      # standard deviations on 1m for x and y, and full revolution for theta
      # setting zero odom_error does NOT give perfect localization
      # (for that you need to choose localization "gps")
      # See https://playerstage.sourceforge.net/doc/Stage-2.0.0/group__model__position.html
      # for more details on the odom_error.

  hokuyoutm30lx (         # laserscan sensor
    pose [ 0.000 0.000 0.000 0.000 ]
  )

  size [ 0.350 0.350 0.250 ]
  origin [ 0.000 0.000 0.000 0.000 ]

  block (
    points 6
    point[0] [0.75 0]
    point[1] [1 0.25]
    point[2] [1 0.75]
    point[3] [0.75 1]
    point[4] [0 1]
    point[5] [0 0]
    z [0 1]
  )

  gui_nose 1
)
