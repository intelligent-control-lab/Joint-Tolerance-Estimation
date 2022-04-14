# Sum-of-Square-Safety-Optimization

By Weiye Zhao, Suqin He, and Changliu Liu

### Introduction
**Sum-of-Square-Safety-Optimization** contains the implementation of JTE algorithms proposed in this paper: https://arxiv.org/abs/2104.08896 

### Requirements: software

0.	MATLAB 2019a or later.

### Requirements: hardware

CPU, Windows 7 or later, MAC OS.

### To recover the results reported in the paper 
0.	Run `test\sixdof.m` to compute the joint tolerance bound for six degrees of freedom robot under different obstacles settings.
0.	Run `test\two_D_plane.m` to compute the joint tolerance bound for 2 degrees of freedom plane robot under different obstacles settings.

### To plot the figures reported in the paper
0.	Run `eval\eval_.m` to plot the sampled end effector position points with respect to the half plane constraints 
