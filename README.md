# KUKA Robot Arm Trajectory
**Objective:** Design a trajectory for the KUKA LBR iiwa 7 R800, a 7 degree-of-freedom (DOF) robotic arm, to place the 
rectangular block attached to its flange on a specified target.

## Contents
* `src`: contains the main script and helper functions used to generate the joint trajectory<br>
  * `DH_mat.m` - computes the homogeneous transformation matrix provided the Denavit-Hartenberg (DH) parameters<br>
  * `an_Jac.m` - calculates the analytical Jacobian given the joint angles, position of the end-effector, and 6x6 transformation matrix, $$T_A$$<br>
  * `fwd_kin.m` - forward kinematics to generate the position and orientation of the end-effector at the current joint configuration<br>
  * `fwd_kin_rect.m` - forward kinematics to generate the position and orientation of the rectangular block at the current joint configuration<br>
  * `main.m` - main script containing both forward and inverse kinematics as well as design of a joint trajectory within the joint's boundaries. Both position and velocity limits are accounted for in addition to obstacle avoidance. The final output is a txt file containing the joint angles.<br>
  * `myrotmat.m` - generates the rotation matrix given a joint angle and axis of rotation<br>
  * `trans_mat_7dof.m` - calculates the final homogeneous transformation matrix from the base to end-effector provided DH parameters. $$\theta$$ and $$\alpha$$ are both 1x7 vectors   <br>
* `Report.pdf` - technical report
* `scully_camryn.txt` - joint angles (in radians) for joints 1 to 7. The files contains 2,000 rows so that the rectangular flange reaches the target in 10 seconds.
