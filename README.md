# Joint-Tolerance-Estimation

By Weiye Zhao, Suqin He, and Changliu Liu

### Introduction
**Joint-Tolerance-Estimation** contains the implementation of JTE algorithms proposed in this paper: https://arxiv.org/abs/2104.08896 (newest version is in submission to CDC 2022)

### Requirements: software

0.	MATLAB 2019a or later.

### Requirements: hardware

CPU, Windows 7 or later, MAC OS.

### To recover the results reported in the paper 
0.	Run `Planar_robot\two_D_plane_*dof.m` to compute the joint tolerance bound for varying degrees of freedom robot in 2d planar plane using JTE algorithm.
0.	Run `Cartesian_robot\cartesian_*dof.m` to compute the joint tolerance bound for varying degrees of freedom plane robot in Cartesian space using JTE algorithm.
0.  Run `adversarial\adversarial_2d_*dof.m` to compute the joint tolerance bound for varying degrees of freedom robot in 2d planar plane using Adversarial Optimization algorithm.
0.  Run `adversarial\adversarial_cartesian_*dof.m` to compute the joint tolerance bound for varying degrees of freedom plane robot in Cartesian space using Adversarial Optimization algorithm.

### To plot the figures reported in the paper
0.	Run `eval\eval_cartesian_*dof.m` to plot the sampled end effector position points with respect to the half plane constraints 
