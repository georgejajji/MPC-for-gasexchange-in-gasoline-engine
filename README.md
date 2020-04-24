# MPC-for-gasexchange-in-gasoline-engine

INTRODUCTION
============
A model predictive controller proposed method for gasoline engines in MATLAB/Simulink
This MPC was developed in MATLAB and Simulink for a master thesis at Linkoping University (ISY). 
The code for MPC functions are coded by Robin Holmbom, Phd Student at Linkoping University.
The structure is partially coded by George Jajji a M.Sc. student at Linkoping University and expanded upon for usage in his master thesis.

The MPC is programmed with symbolic expressions in MATLAB and is runned in Simulink. To be able to run the Simulink model you need to have qpOASES installed. qpOASES is an open-source QP-solver developed with MPC purposes in mind. It has a Simulink interface which is used in the thesis. qpOASES can be found at https://github.com/coin-or/qpOASES, with full manual and installation guides.

GETTING STARTED
===============
1. Install qpOASES and add it to your working folder or MATLAB path.

2. Open the main.m and run the script. Matlab functions should be created in the folder.
 
3. Open the main.slx model in Simulink and run. 

The MPC can be configurated easily in the first section of main.m file. Adding new states or inputs requires for expansion of the state model by coding the dynamics.
Furthermore the cost-function is determined by which states are included in the matrix  M. For detailed explanation of how to change constraints and how to formulate the cost-function see Master Thesis of George Jajji.
