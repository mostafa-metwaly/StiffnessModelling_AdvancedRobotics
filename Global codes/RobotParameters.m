%% Robot Parameters:

I = eye(6);


%% Robot Links Length:
%number of links and length of each link:
Robot.Links = [0, 10, 10 ,0 ];
Robot.Joint_no = 4;


%% Description of model components: 
% RigidLinks , FlexibleLinks
% ActiveJoints , PassiveJoints , RigidJoints
Robot.descriptionLink = ["RigidLinks","FlexibleLinks","FlexibleLinks","RigidLinks"];
Robot.descriptionJoint = ["ActiveJoints","PassiveJoints","PassiveJoints","PassiveJoints"];

%% Joint Types:
% joint_type = Revolute;(1,2,3)
% Joint_axis = "X";(1)
% joint_type = Prismatic;(4,5,6)
% Joint_axis = "X";(4)
Robot.JointType = [1,4,4,4];
Robot.numnodes = 9;


%% Stiffness Parameters:
Robot.E = 7*10^10 ;
Robot.G = 2.55*10^10;
Robot.Ka = 1000000;

% robot links beam type[ cylinder, HollowCylinder, rectangle] :
Robot.LinkType = "cylinder";
% Robot links diameter in case of cylinder beam
Robot.D = 0.15;
% Robot links inner diameter(D(1)) and outer diameter(D(2)) in case of hollow cylinder beam
% Robot.D = [0.2 ,0.35];
% Robot links base and Heigth in case of rectangle beam where D(1) is base
% and D(2) is the Heigth
% Robot.D = [0.2 ,0.35];



%% Robot active joint stiffness 
Robot.Ka = 1000000;




