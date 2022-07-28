# Robotics_Project_1

## What is
This is the repository which contains the workspace of the first project of the course **Robotics** at Politecnico di Milano.

University: **Politecnico di Milano, Laurea Magistrale (MSc) - Computer Science and Engineering**

## The project
We had to develop this project with ROS, and we had some bags recorded.
The robot with which were recorded, was an omnidirectional robot with mecanum wheels (4 wheels with rollers at 45°). We had encoders on each wheel, which were giving to us RPMs (N.B.: they were not classic RPMs, in this case the acronym meaning was Radiants Per Minute), and Ticks of the encoder. The RPMs were really noisy, while ticks were more accurate. We were given the wheels radious, the wheel position along x (half length), and the wheel position along y (half width). Moreover, we had a ground truth pose of the robot acquired with OptiTrack.

The goal was to:
1. Compute odometry using appropriate kinematics
	- compute robot linear and angular velocities (v, omega) from wheel encoders;
	- compute odometry using both Euler and Runge-Kutta integration
		- ROS parameters for initial pose
	- Calibrate (fine - tune) robot parameters to match ground truth
2. Compute wheels control speeds from v, omega;
3. Add a service to reset the odometry to a specified pose (x,y,theta)
4. Use dynamic reconfigure to select between integration method (using an enum with two values: *Euler*, *RK*)

In particular, in point 1 we had to publish v, omega as topic `cmd_vel` of type `geometry_msgs/TwistStamped`, and once computed odometry we had to publish it as `nav_msgs/Odometry` on topic `odom`, and broadcast TF `odom->base_link`.

In point 2 we hade to compute wheels speeds (RPM) from v, omega, and publish them as custom message on topic `wheels_rpm`. The custom message needed to have as prototype:

`
Header header
`</br>
`
float64 rpm_fl
`</br>
`
float64 rpm_fr
`</br>
`
float64 rpm_rr
`</br>
`
float64 rpm_rl
`</br>

where `f/r` stands for front/rear, and `l/r` for left/right. Then we had to check that the results matched the encoders value, apart from some noise.This is useful to check that the results match the recorded encoder values (using for example `rqt_plot` or `plotjugler`)

Initial data:

- Wheel radious
- Wheel position along x
- Wheel position along y
- Gear ratio

Given bags:

- bag 1 with only linear motions (no rotations)
- bag 2 with only forard motions and rotations (no y translation)
- bag 2 with a "freestyle" recording

## Our Team
- Luca Alessanrini  
- Massimo Buono (added as collaborator)
- Mattia Portanti (added as collaborator)

## How we divided the work
We decided to face this challenge for the most part with in-presence meetings, where we discussed and implemented together different choices

## How to replicate our results
We developed this on Ubuntu 18, with ROS Melodic. Once having downloaded the workspace, it is necessary to link it into the bashrc file and build it in order to use the project.

## Submission
We had to submit not the whole workspace, but just only our package.
