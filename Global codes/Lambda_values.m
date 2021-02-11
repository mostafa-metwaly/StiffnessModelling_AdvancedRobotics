% Lambdas :
function [lambda_r, lambda_p, lambda_e]= Lambda_values(JointType,AllNodes)
    %% General case
    lambda_rw = [];
    lambda_pw = [];
    lambda_ew = [];
    i = 0;
    num = length(JointType) ;

    for a = 1 : length(AllNodes)
        if (AllNodes(a) ~= 0)
            i = i+ 1
            
            
            lambda_rt = eye(6);lambda_rt(JointType(i),:)=[];
            lambda_pt = zeros(1,6);lambda_pt(JointType(i)) = 1;
            lambda_et = zeros(1,6);lambda_et(JointType(i)) = 1;

            lambda_rw = [lambda_rw ;lambda_rt];
            lambda_pw = [lambda_pw ;lambda_pt];
            lambda_ew = [lambda_ew ;lambda_et];

        end

    end

    

    rows_vec = ones(1,num);
    lambda_r = mat2cell(lambda_rw ,[1 1 1 1] * 5);
    lambda_p = mat2cell(lambda_pw ,rows_vec);
    lambda_e = mat2cell(lambda_ew ,rows_vec);

end


% %% Lambda_rigid
% % Lambda_rigid in prismatic joints
% lambda_r_xp = eye(6);
% lambda_r_xp(1,:) = [];
% 
% lambda_r_yp = eye(6);
% lambda_r_yp(2,:) = [];
% 
% lambda_r_zp = eye(6);
% lambda_r_zp(3,:) = [];
% 
% 
% % Lambda_rigid in Revolute joints
% lambda_r_xr = eye(6);
% lambda_r_xr(4,:) = [];
% 
% lambda_r_yr = eye(6);
% lambda_r_yr(5,:) = [];
% 
% lambda_r_zr = eye(6);
% lambda_r_zr(6,:) = [];
% 
% 
% %% Lambda_Passive:
% % Lambda_Passive in prismatic joints
% lambda_p_xp = zeros(1,6);
% lambda_p_xp(1) = 1;
% 
% lambda_p_yp = zeros(1,6);
% lambda_p_yp(2) = 1;
% 
% lambda_p_zp = zeros(1,6);
% lambda_p_zp(3) = 1;
% 
% 
% % Lambda_Passive in Revolute joints
% lambda_p_xr = zeros(1,6);
% lambda_p_xr(4) = 1;
% 
% lambda_p_yr = zeros(1,6);
% lambda_p_yr(5) = 1;
% 
% lambda_p_zr = zeros(1,6);
% lambda_p_zr(6) = 1;
% 
% 
% %% Lambda_Elastic:
% % Lambda_Elastic in prismatic joints
% lambda_e_xp = zeros(1,6);
% lambda_e_xp(1) = 1;
% 
% lambda_e_yp = zeros(1,6);
% lambda_e_yp(2) = 1;
% 
% lambda_e_zp = zeros(1,6);
% lambda_e_zp(3) = 1;
% 
% 
% % Lambda_Elastic in Revolute joints
% lambda_e_xr = zeros(1,6);
% lambda_e_xr(4) = 1;
% 
% lambda_e_yr = zeros(1,6);
% lambda_e_yr(5) = 1;
% 
% lambda_e_zr = zeros(1,6);
% lambda_e_zr(6) = 1;
% 
% 
