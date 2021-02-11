%% Joints 
function [ jointEqJT ] = Joint_Equations(AllNodes,descriptionJoint,JointType,Ka)

RobotParameters; 
AllNodes = Robot.Nodes;

i = 0;
jointEqJT = [];
jointEq = [];

% Parameters lambdas
[lambda_ra,lambda_pa,lambda_ea] = Lambda_values(Robot.JointType,AllNodes)

numnodes = length(AllNodes);

for a =1 : numnodes
    if (AllNodes(a) ~= 0)
        i = i+ 1;
        node = i; nodef = i+1;

        lambda_r = cell2mat(lambda_ra(i));
        lambda_p = cell2mat(lambda_pa(i));
        lambda_e = cell2mat(lambda_ea(i));


        %% Joints:    
        %% Rigid Joint
        if (descriptionJoint(i) == "RigidJoints")

            jointEq = mat2cell(zeros(12,2*numnodes*6),[6 6],ones(1,numnodes*2)*6);
            jointEq(1,node+numnodes) = mat2cell(I,[6],[6]);
            jointEq(1,nodef+numnodes) = mat2cell(-I,[6],[6]);
            jointEq(2,node) = mat2cell(I,[6],[6]);
            jointEq(2,nodef) = mat2cell(I,[6],[6]);


        %% Passive Joint
        elseif (descriptionJoint(i) == "PassiveJoints")

            jointEq = mat2cell(zeros(12,2*numnodes*6),[5 5 1 1],ones(1,numnodes*2)*6);
            jointEq(1,node+numnodes) = mat2cell(lambda_r,[5],[6]);
            jointEq(1,nodef+numnodes) = mat2cell(-lambda_r,[5],[6]);
            jointEq(2,node) = mat2cell(lambda_r,[5],[6]);
            jointEq(2,nodef) = mat2cell(lambda_r,[5],[6]);
            jointEq(3,node) = mat2cell(lambda_p,[1],[6]);
            jointEq(4,nodef) = mat2cell(lambda_p,[1],[6]);


        %% Active (Elastic) Joint
        elseif (descriptionJoint(i) == "ActiveJoints")

            jointEq = mat2cell(zeros(12,2*numnodes*6),[5 6 1],ones(1,numnodes*2)*6);
            jointEq(1,node+numnodes) = mat2cell(lambda_r,[5],[6]);
            jointEq(1,nodef+numnodes) = mat2cell(-lambda_r,[5],[6]);
            jointEq(2,node) = mat2cell(I,[6],[6]);
            jointEq(2,nodef) = mat2cell(I,[6],[6]);
            jointEq(3,node) = mat2cell(lambda_e,[1],[6]);
            jointEq(3,node+numnodes) = mat2cell(Ka*lambda_e,[1],[6]);
            jointEq(3,nodef+numnodes) = mat2cell(-Ka*lambda_e,[1],[6]);

        end

        jointEqJT = [jointEqJT ;jointEq];
       
    end

        
end