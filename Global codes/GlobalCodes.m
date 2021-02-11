% Global Codes Main
close all;
clear all;
clc;

% include all 'Robot' structure with all constants and parameters
RobotParameters; 

%% Start building the MSA model: 

%Joints Matricies(Active,passive,Rigid)  
JointEq = Joint_Equations(Robot.Nodes,Robot.descriptionJoint,Robot.JointType,Robot.Ka)

%Links Matricies(Rigid ,Flexible)
LinkEq = Link_Equations(Robot.Nodes,Robot.descriptionLink,Robot.G,Robot.E,Robot.Links ,Robot.D,Robot.LinkType)

%Aggregation Matrix 
[Kc ,S] = Aggregated_Matrix(JointEq,LinkEq)

%Delta_t Deflection calculation



