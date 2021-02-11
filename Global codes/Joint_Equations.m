%% Joints 
function [ jointEq] = Joint_Equations(node,nodef,numnodes,I,descriptionJoint,lambda_r,lambda_p,lambda_e,Ka)


    
    lambda_r = cell2mat(lambda_r(1));
    lambda_p = cell2mat(lambda_p(1));
    lambda_e = cell2mat(lambda_e(1));
    
    %% Joints:    
    %% Rigid Joint
    if (descriptionJoint == "RigidJoints")
        
        jointEq = mat2cell(zeros(12,2*numnodes*6),[6 6],ones(1,numnodes*2)*6);
        jointEq(1,node+numnodes) = mat2cell(I,[6],[6]);
        jointEq(1,nodef+numnodes) = mat2cell(-I,[6],[6]);
        jointEq(2,node) = mat2cell(I,[6],[6]);
        jointEq(2,nodef) = mat2cell(I,[6],[6]);

    
    %% Passive Joint
    elseif (descriptionJoint == "PassiveJoints")
        
        jointEq = mat2cell(zeros(12,2*numnodes*6),[5 5 1 1],ones(1,numnodes*2)*6);
        jointEq(1,node+numnodes) = mat2cell(lambda_r,[5],[6]);
        jointEq(1,nodef+numnodes) = mat2cell(-lambda_r,[5],[6]);
        jointEq(2,node) = mat2cell(lambda_r,[5],[6]);
        jointEq(2,nodef) = mat2cell(lambda_r,[5],[6]);
        jointEq(3,node) = mat2cell(lambda_p,[1],[6]);
        jointEq(4,nodef) = mat2cell(lambda_p,[1],[6]);


    %% Active (Elastic) Joint
    elseif (descriptionJoint == "ActiveJoints")
        
        jointEq = mat2cell(zeros(12,2*numnodes*6),[5 6 1],ones(1,numnodes*2)*6);
        jointEq(1,node+numnodes) = mat2cell(lambda_r,[5],[6]);
        jointEq(1,nodef+numnodes) = mat2cell(-lambda_r,[5],[6]);
        jointEq(2,node) = mat2cell(I,[6],[6]);
        jointEq(2,nodef) = mat2cell(I,[6],[6]);
        jointEq(3,node) = mat2cell(lambda_e,[1],[6]);
        jointEq(3,node+numnodes) = mat2cell(Ka*lambda_e,[1],[6]);
        jointEq(3,nodef+numnodes) = mat2cell(-Ka*lambda_e,[1],[6]);

    end




end