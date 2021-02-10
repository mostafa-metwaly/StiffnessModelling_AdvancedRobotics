% Global Codes Main
close all;
clear all;
clc;

% include all 'Robot' structure with all constants and parameters
RobotParameters; 


%% Start building the MSA model: 
% Each link

% Stiffness Matrix for each link
[K_11,k_12,k_21,k_22] = StiffnessMatrix(Robot.G,Robot.E,Robot.Links ,Robot.D,Robot.LinkType)

% Parameters lambdas
[lambda_r,lambda_p,lambda_e] = Lambda_values(Robot.JointType)

%Joints Matricies(Active,passive,Rigid)
jointEq1 =Joint_Equations(1,2,Robot.numnodes,I,Robot.description,lambda_r,lambda_p,lambda_e)
jointEq2 =Joint_Equations(3,4,Robot.numnodes,I,Robot.description,lambda_r,lambda_p,lambda_e)
jointEq3 =Joint_Equations(5,6,Robot.numnodes,I,Robot.description,lambda_r,lambda_p,lambda_e)
jointEq4 =Joint_Equations(7,8,Robot.numnodes,I,Robot.description,lambda_r,lambda_p,lambda_e)

jointEq = [jointEq1 ;jointEq2 ;jointEq3 ;jointEq4 ]
A = cell2mat(jointEq) 
%Aggregation Matrix 
% []=Aggregated_Matrix()

%Delta_t Deflection calculation



