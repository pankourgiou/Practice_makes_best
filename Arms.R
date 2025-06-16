library(rgl)

# Function to create a rotation matrix for a joint
rotation_matrix <- function(rx, ry, rz) {
  Rx <- matrix(c(1, 0, 0,
                 0, cos(rx), -sin(rx),
                 0, sin(rx), cos(rx)), nrow = 3, byrow = TRUE)
  
  Ry <- matrix(c(cos(ry), 0, sin(ry),
                 0, 1, 0,
                 -sin(ry), 0, cos(ry)), nrow = 3, byrow = TRUE)
  
  Rz <- matrix(c(cos(rz), -sin(rz), 0,
                 sin(rz), cos(rz), 0,
                 0, 0, 1), nrow = 3, byrow = TRUE)
  
  return(Rz %*% Ry %*% Rx)
}

# Function to compute forward kinematics for a 3-joint arm
forward_kinematics_3d <- function(joint_angles, segment_lengths, origin = c(0, 0, 0)) {
  pos <- matrix(origin, nrow = 3)
  points <- matrix(origin, ncol = 1)
  
  R <- diag(3)
  for (i in 1:3) {
    R <- R %*% rotation_matrix(joint_angles[i,1], joint_angles[i,2], joint_angles[i,3])
    next_point <- pos + R %*% c(segment_lengths[i], 0, 0)
    points <- cbind(points, next_point)
    pos <- next_point
  }
  
  return(points)
}

# Define arm parameters
segment_lengths <- c(1, 0.8, 0.6)

# Define joint angles for both arms (rx, ry, rz in radians)
arm1_angles <- matrix(c(pi/6, pi/6, 0,
                        pi/6, 0, pi/4,
                        0, pi/6, pi/6), ncol = 3, byrow = TRUE)

arm2_angles <- matrix(c(-pi/6, pi/4, 0,
                        pi/8, pi/8, -pi/6,
                        0, -pi/6, pi/4), ncol = 3, byrow = TRUE)

# Compute positions
arm1_points <- forward_kinematics_3d(arm1_angles, segment_lengths, origin = c(0, 0, 0))
arm2_points <- forward_kinematics_3d(arm2_angles, segment_lengths, origin = c(2, 0, 0))

# Plot arms in 3D
open3d()
lines3d(t(arm1_points), col = "red", lwd = 3)
spheres3d(t(arm1_points), radius = 0.05, col = "red")

lines3d(t(arm2_points), col = "blue", lwd = 3)
spheres3d(t(arm2_points), radius = 0.05, col = "blue")

axes3d()
title3d("3D Robotic Arms Simulation", xlab="X", ylab="Y", zlab="Z")
