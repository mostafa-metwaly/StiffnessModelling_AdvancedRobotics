clearvars -except all_deflections_with_forces_MSA MSA_time
% close all
clc
% Forward Kinematics:
% Define manipulator parameters

% q_a -> active joint value
% theta -> virtual joints
% q_p -> passive joint
% space_* -> the workspace dimensions
% platform_link -> Rigid platform � distance from corners to center
% syms q_active_z q_active_y q_active_x theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9 theta10 theta11 theta12 theta 13 q_p1 q_p2 q_p3 space_z space_y space_x platform_link link1 link2

% thetas = [theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9 theta10 theta11 theta12 theta 13];
% q_passive = [q_p1 q_p2 q_p3];
% links = [link1 link2];

space_z = 1;
space_y = 1;
space_x = 1;
thetas = zeros(1,13);
platform_link = 0.1;


% T_global_leg = T_base_leg * T_local_leg * T_tool

T_base_z = eye(4); % Transforming form the global frame to the z_leg local frame
T_base_y =Tz(space_z)*Rx(-pi/2); % Transforming form the global frame to the y_leg local frame
T_base_x = Ty(space_y)*Ry(pi/2)*Rz(pi); % Transforming form the global frame to the x_leg local frame
T_base(:,:,1) = T_base_x;
T_base(:,:,2) = T_base_y;
T_base(:,:,3) = T_base_z;

% For the platform it should be
T_tool_z = [T_base_z(1:3,1:3)' zeros(3,1); 0 0 0 1]*Ty(platform_link);
T_tool_y = [T_base_y(1:3,1:3)' zeros(3,1); 0 0 0 1]*Rz(-pi/6)*Tx(platform_link)*Rz(-pi/6)';
T_tool_x = [T_base_x(1:3,1:3)' zeros(3,1); 0 0 0 1]*Rz(pi/6)*Tx(-platform_link)*Rz(pi/6)';
T_tool(:,:,1) = T_tool_x; 
T_tool(:,:,2) = T_tool_y;
T_tool(:,:,3) = T_tool_z;

% Stiffness matrices:

E = 7*10^10 ;
G = 2.55*10^10;
D = 0.15;
R = 0.075;
A = pi*D^2/4;
Iy = (pi*D^4)/64;
Iz = (pi*D^4)/64;
Ip = Iz+Iy;
K_active = 1000000; 
L = 1;
links = [L, L];

K_22=[E*A/L 0 0 0 0 0; ...
      0 12*E*Iz/L^3 0 0 0 -6*E*Iz/L^2; ...
      0 0 12*E*Iy/L^3 0 6*E*Iy/L^2 0; ...
      0 0 0 G*Ip/L 0 0; ...
      0 0 6*E*Iy/L^2 0 4*E*Iy/L 0; ...
      0 -6*E*Iz/L^2 0 0 0 4*E*Iz/L];

K_22_5 = K_22;
K_22_3 = K_22_5;

% Calculating T_leg using VJM Model

% function T_leg = VJM_T_leg(T_base, T_tool, q_active, thetas, q_passive, links)
% % ** This function calculates the forward kinematics for the given leg ** %
% 
% % Structure for each leg (locally):
% % 1 - active joint (q_a)
% % 2 - 1 dof virtual spring (theta1) along the active joint axis (local z)
% % 3 - passive joint (q_p1)
% % 4 - rigid link1
% % 5 - 6 dof virtual spring (theta2 - theta7)
% % 6 - passive joint (q_p2)
% % 7 - rigid link2
% % 8 - 6 dof virtual spring (theta8 - theta13)
% % 9 - passive joint (q_p3)
% 
% T_leg = T_base * (Tz(q_active) * Tz(theta[1]) * Rz(q_p[1]) * Tx(link[1]) * ( Tx(theta[2])*Ty(theta[3])*Tz(theta[4])*Rx(theta[5])*Ry(theta[6])*Rz(theta[7]) ) * Rz(q_p[2]) * Tx(link[2]) * ( Tx(theta[8])*Ty(theta[9])*Tz(theta[10])*Rx(theta[11])*Ry(theta[12])*Rz(theta[13]) ) * Rz(q_p[3]) ) * T_tool;
% end

% Measuring Deflections:
% Get passive and active joint values:

start = 0.1; % the end-effector will start from the point (x = start, y = start, z = start)
step = 0.1; % increment each aixs by the "step" value every loop
stop = 1; % stop at (x = stop, y = stop, z = stop)

%% Force Applied:

F_x= [100 0 0 0 0 0];
F_y= [0 100 0 0 0 0];
F_z= [0 0 100 0 0 0];
Ft = [F_x ; F_y ; F_z];

all_deflections_with_forces_VJM = {};

tic;
for f = 1:3 
    F = transpose(Ft(f,:));
    
    all_deflections_VJM = [];
    x_all = [];
    y_all = [];
    z_all = [];
    for x = start:step:stop
        for y = start:step:stop
         for z = start:step:stop
            z_all = [z_all z];
            y_all = [y_all y];
            x_all = [x_all x];
            end_effector_possition = [x y z];
            % -------------------------------------------------------------
            % 1 - calculate the passive and active joints values:
            [q_passive_x, q_actives] = IK_tri(T_base_x, T_tool_x, end_effector_possition, platform_link, links);
            q_active_x = q_actives(1);
            [q_passive_y, q_actives] = IK_tri(T_base_y, T_tool_y, end_effector_possition, platform_link, links);
            q_active_y = q_actives(2);
            [q_passive_z, q_actives] = IK_tri(T_base_z, T_tool_z, end_effector_possition, platform_link, links);
            q_active_z = q_actives(3);
            
            % -------------------------------------------------------------
            % 2 - calculate the jacobian w.r.t the passive joints:
            passive_jacobian_x = Leg_passive_jacobian(T_base_x, T_tool_x, q_active_x, q_passive_x, thetas, links);
            passive_jacobian_y = Leg_passive_jacobian(T_base_y, T_tool_y, q_active_y, q_passive_y, thetas, links);
            passive_jacobian_z = Leg_passive_jacobian(T_base_z, T_tool_z, q_active_z, q_passive_z, thetas, links);
            
            J_q = {passive_jacobian_x ; passive_jacobian_y ;passive_jacobian_z};

            % -------------------------------------------------------------
            % 3 - calculate the jacobian w.r.t the virtual joints:
            theta_jacobian_x = Jacob_theta_leg(T_base_x, T_tool_x, q_active_x, q_passive_x, thetas, links);
            theta_jacobian_y = Jacob_theta_leg(T_base_y, T_tool_y, q_active_y, q_passive_y, thetas, links);
            theta_jacobian_z = Jacob_theta_leg(T_base_z, T_tool_z, q_active_z, q_passive_z, thetas, links);
            

            J_theta = {theta_jacobian_x ; theta_jacobian_y ; theta_jacobian_z};
            % -------------------------------------------------------------
            % 4 - calculate stiffness of virtual joints:
            K_theta = K_theta_VJM(K_active,K_22_3,K_22_5);
            K_theta = {K_theta; K_theta; K_theta};
            
            % 5 - calculate overall stiffness
            Kc_final = Kc_VJM(K_theta,J_theta,J_q);
            
            % 6 - calculate and store deflection in the end-effector:
            dt = inv(Kc_final)*F;
            all_deflections_VJM = [all_deflections_VJM dt];
        end
    end
end
all_deflections_with_forces_VJM{end+1} = all_deflections_VJM;
def_mag_VJM = (all_deflections_VJM(1,:).^2 + all_deflections_VJM(2,:).^2 + all_deflections_VJM(3,:).^2).^0.5;

%%printing:

figure;
titles = ['x', 'y', 'z'];
scatter3(x_all, y_all, z_all, 40,def_mag_VJM,'filled')    % draw the scatter plot
ax = gca;
ax.XDir = 'reverse';
%     view(-31,14)
xlabel("X")
ylabel("Y")
zlabel("Z")
title(sprintf(" VJM Method Deflections due to force in %s directions",titles(f)))

colormap(hot);
cb = colorbar;                                     % create and label the colorbar
cb.Label.String = 'Deflections Magnitude';
end

VJM_time = toc;