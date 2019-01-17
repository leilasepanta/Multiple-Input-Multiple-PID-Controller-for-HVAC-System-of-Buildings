# MIMO PID Controller for HVAC System of Buildings


What is this about?
===================

In this repository you can find codes for an advanced controller that has been designed for HVAC systems in smart buildings. In this case I have designed and programmed the controller for a two-storied building with 6 rooms on first floor and 4 rooms on second floor.  In the file MIMO_PID_HVAC.m you will be able to find the code after initialization. In the beginning the thermal model of the building has been converted to its electrical representation and the values in the initialization for R and C are the values that have been derived from that conversion. For the mathematical model of the building I have used State Space Representation and the matrices had been derived from the initialization. Based on your model you can organize the equations of the physical model in the state space representation form and by comparison you will be able to derive the A, B, C, and D matrices. In this model I have considered the disturbance of the system and the measurement noise as well. However, you can see that the model works very well and the controller helps the HVAC system follow the desired trajectory which is temperature of 298.15 for each of the rooms. This model works for any temperature and any schedule for the temperature, which means the desired trajectory can be a constant, a linear or a nonlinear function. In the function plant you can see the internal and external disturbances of the building.  Designing Multiple Input Multiple Output Systems can be tough. I hope this code helps the ones who are doing researches in this area.

Figure 1 shows how the system works with the PID controller.
![figure 1](https://user-images.githubusercontent.com/35972214/51286252-2d8ad800-19c0-11e9-9cce-0673aa413ea0.JPG)

How does it work?
================

1.	Clone or download the files
2.	.First run the plant file and it will give you an error, since it is just a function handler. 
3.	After that, run the other file and it will show you plots for each of the rooms temperatures, and the changes that the controller goes through to follow the desired temperature.
4.	Give this GitHub repo a star 
Don’t Forget :You should run both files simultaneously

Copy Right
==========
See LICENSE for details. Copyright (c) 2019 Leila Amanzadeh.

