% Global Codes Main
close all;
clear all;
clc;

% include all 'Robot' structure with all constants and parameters
RobotParameters; 

%% Start building the MSA model: 
% Each link

% Stiffness Matrix for each link
[K_11,K_12,K_21,K_22] = StiffnessMatrix(Robot.G,Robot.E,Robot.Links ,Robot.D,Robot.LinkType)

% Parameters lambdas
[lambda_r,lambda_p,lambda_e] = Lambda_values(Robot.JointType)

%Joints Matricies(Active,passive,Rigid)
jointEq1 = Joint_Equations(2,3,Robot.numnodes,I,Robot.descriptionJoint(1),lambda_r,lambda_p,lambda_e,Robot.Ka)
jointEq2 = Joint_Equations(4,5,Robot.numnodes,I,Robot.descriptionJoint(2),lambda_r,lambda_p,lambda_e,Robot.Ka)
jointEq3 = Joint_Equations(6,7,Robot.numnodes,I,Robot.descriptionJoint(3),lambda_r,lambda_p,lambda_e,Robot.Ka)
jointEq4 = Joint_Equations(8,9,Robot.numnodes,I,Robot.descriptionJoint(4),lambda_r,lambda_p,lambda_e,Robot.Ka)

LinkEq11 = Link_Equations(1,2,Robot.numnodes,I,Robot.descriptionLink(1), K_11, K_12, K_21, K_22)
LinkEq12 = Link_Equations(5,6,Robot.numnodes,I,Robot.descriptionLink(2), K_11, K_12, K_21, K_22)
LinkEq13 = Link_Equations(7,8,Robot.numnodes,I,Robot.descriptionLink(3), K_11, K_12, K_21, K_22)

LinkEqT = [jointEq1;jointEq2;jointEq3;jointEq4;LinkEq11;LinkEq12;LinkEq13]
%Aggregation Matrix 
Kc = Aggregated_Matrix(LinkEqT,I)

%Delta_t Deflection calculation



