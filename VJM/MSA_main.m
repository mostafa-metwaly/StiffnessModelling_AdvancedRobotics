
clc;
clear all;
close all;

%% constants:

Kc_all=0;   
E = 7*10^10 ;
G = 2.55*10^10;
L = 1;
D = 0.15;
R = 0.075;
A = pi*D^2/4;
Ka = 1000000;
Iy = (pi*D^4)/64;
Iz = (pi*D^4)/64;
Ip = Iz+Iy;
I=eye(6);


%% Platform Link
platform_link = 0.1;
d8e_z = [0, platform_link, 0];
d8e_y = [platform_link*cosd(30), platform_link*cosd(-120),0];
d8e_x = [platform_link*cosd(30+180), platform_link*cosd(120),0];
d8ee = [d8e_x ; d8e_y ; d8e_z];


%% Stiffness Matrix for each link

K_11=[E*A/L 0 0 0 0 0;
  0 12*E*Iz/L^3 0 0 0 6*E*Iz/L^2;
  0 0 12*E*Iy/L^3 0 -6*E*Iy/L^2 0;
  0 0 0 G*Ip/L 0 0;
  0 0 -6*E*Iy/L^2 0 4*E*Iy/L 0;
  0 6*E*Iz/L^2 0 0 0 4*E*Iz/L];

K_12=[-E*A/L 0 0 0 0 0;
  0 -12*E*Iz/L^3 0 0 0 6*E*Iz/L^2;
  0 0 -12*E*Iy/L^3 0 -6*E*Iy/L^2 0;
  0 0 0 -G*Ip/L 0 0;
  0 0 6*E*Iy/L^2 0 2*E*Iy/L 0;
  0 -6*E*Iz/L^2 0 0 0 2*E*Iz/L];

K_21=transpose(K_12);

K_22=[E*A/L 0 0 0 0 0;
  0 12*E*Iz/L^3 0 0 0 -6*E*Iz/L^2;
  0 0 12*E*Iy/L^3 0 6*E*Iy/L^2 0;
  0 0 0 G*Ip/L 0 0;
  0 0 6*E*Iy/L^2 0 4*E*Iy/L 0;
  0 -6*E*Iz/L^2 0 0 0 4*E*Iz/L];


K_11_3=K_11;
K_12_3=K_12;
K_21_3=K_21;
K_22_3=K_22;
K_11_5=K_11;
K_12_5=K_12;
K_21_5=K_21;
K_22_5=K_22; 


%% Active Elastic Joint <1,2>:

lambda_r_12_x=[0 1 0 0 0 0;
               0 0 1 0 0 0;
               0 0 0 1 0 0;
               0 0 0 0 1 0;
               0 0 0 0 0 1];

lambda_r_12_y=[1 0 0 0 0 0;
               0 0 1 0 0 0;
               0 0 0 1 0 0;
               0 0 0 0 1 0;
               0 0 0 0 0 1];


lambda_r_12_z=[1 0 0 0 0 0;
               0 1 0 0 0 0;
               0 0 0 1 0 0;
               0 0 0 0 1 0;
               0 0 0 0 0 1];

lambda_e_12_x=[1 0 0 0 0 0];
lambda_e_12_y=[0 1 0 0 0 0];
lambda_e_12_z=[0 0 1 0 0 0];

lambda_r_12_a = { lambda_r_12_x ; lambda_r_12_y ; lambda_r_12_z };
lambda_e_12_a = { lambda_e_12_x ; lambda_e_12_y ; lambda_e_12_z };

%% Passive joints <3,4>,<5,6>,<7,8>:

lambda_r_34_x=[1 0 0 0 0 0;
               0 1 0 0 0 0;
               0 0 1 0 0 0;
               0 0 0 0 1 0;
               0 0 0 0 0 1];

lambda_r_34_y=[1 0 0 0 0 0;
               0 1 0 0 0 0;
               0 0 1 0 0 0;
               0 0 0 1 0 0;
               0 0 0 0 0 1];

lambda_r_34_z=[1 0 0 0 0 0;
               0 1 0 0 0 0;
               0 0 1 0 0 0;
               0 0 0 1 0 0;
               0 0 0 0 1 0];

lambda_p_34_x=[0 0 0 1 0 0];
lambda_p_34_y=[0 0 0 0 1 0];
lambda_p_34_z=[0 0 0 0 0 1];


lambda_r_34_a={lambda_r_34_x ; lambda_r_34_y ;lambda_r_34_z };
lambda_p_34_a={lambda_p_34_x ; lambda_p_34_y ;lambda_p_34_z };

%% workspace

Square=[];
space_x = 1;
space_y = 1;
space_z = 1; % workspace size
link = [1, 1]; % links length

%% Kinematics solving :

% T_global_leg = T_base_leg * T_local_leg * T_tool

T_base_z = eye(4); % Transforming form the global frame to the z_leg local frame
T_base_y =Tz(space_z)*Rx(-pi/2); % Transforming form the global frame to the y_leg local frame
T_base_x = Ty(space_y)*Ry(pi/2)*Rz(pi); % Transforming form the global frame to the x_leg local frame
T_base(:,:,1) = T_base_x;
T_base(:,:,2) = T_base_y;
T_base(:,:,3) = T_base_z;

% ########################## Change T_tool to adapt with the platform ##########################
% T_tool_z = [T_base_z(1:3,1:3)' zeros(3,1); 0 0 0 1];
% T_tool_y = [T_base_y(1:3,1:3)' zeros(3,1); 0 0 0 1];
% T_tool_x = [T_base_x(1:3,1:3)' zeros(3,1); 0 0 0 1];
% T_tool(:,:,1) = T_tool_x; 
% T_tool(:,:,2) = T_tool_y;
% T_tool(:,:,3) = T_tool_z;

% For the platform it should be
T_tool_z = [T_base_z(1:3,1:3)' zeros(3,1); 0 0 0 1]*Ty(platform_link);
T_tool_y = [T_base_y(1:3,1:3)' zeros(3,1); 0 0 0 1]*Rz(-pi/6)*Tx(platform_link)*Rz(-pi/6)';
T_tool_x = [T_base_x(1:3,1:3)' zeros(3,1); 0 0 0 1]*Rz(pi/6)*Tx(-platform_link)*Rz(pi/6)';
T_tool(:,:,1) = T_tool_x; 
T_tool(:,:,2) = T_tool_y;
T_tool(:,:,3) = T_tool_z;

% number_of_points = fix((movement_end - movement_start)/step + 1)^3
start = 0.1; % the end-effector will start from the point (x = start, y = start, z = start)
step = 0.1; % increment each aixs by the "step" value every loop
stop = 1; % stop at (x = stop, y = stop, z = stop)

% all_deflections = [];

%% Force Applied:

F_x= [100 0 0 0 0 0];
F_y= [0 100 0 0 0 0];
F_z= [0 0 100 0 0 0];
Ft = [F_x ; F_y ; F_z];


all_deflections_with_forces_MSA = {};

%% Aggregating all functions:
tic;
for f = 1:3 
    F = transpose(Ft(f,:));
    all_deflections_MSA = [];
    all_deflections_x = [];
    all_deflections_y = [];
    all_deflections_z = [];
    counter = 0;
    
    x_all = [];
    y_all = [];
    z_all = [];
    
    

    for x = start:step:stop
        for y = start:step:stop
            for z = start:step:stop
                z_all(length(z_all)+1) = z;
                y_all(length(y_all)+1) = y;
                x_all(length(x_all)+1) = x;
                counter = counter + 1;
                

                end_effector_possition = [x, y, z];
                   
                q_passive_x = IK_tri(T_base_x, T_tool_x, end_effector_possition, platform_link, link);
                q_passive_y = IK_tri(T_base_y, T_tool_y, end_effector_possition, platform_link, link);
                q_passive_z = IK_tri(T_base_z, T_tool_z, end_effector_possition, platform_link, link);
                q_passive = [q_passive_x; q_passive_y; q_passive_z];
                
                [Q1_a, Q2_a] = transformStiffness(T_base, end_effector_possition, q_passive, link);



    %Aggregating all the constraints and links and joints:

                for i= 1:3
    % where i is refered to each axis of rotation 1=x, 2=y, 3=z

                    Q1 = Q1_a(:,:,i);
                    K_11_3=Q1*K_11*transpose(Q1);
                    K_12_3=Q1*K_12*transpose(Q1);
                    K_21_3=Q1*K_21*transpose(Q1);
                    K_22_3=Q1*K_22*transpose(Q1);

                    Q2 = Q2_a(:,:,i);
                    K_11_5=Q2*K_11*transpose(Q2);
                    K_12_5=Q2*K_12*transpose(Q2);
                    K_21_5=Q2*K_21*transpose(Q2);
                    K_22_5=Q2*K_22*transpose(Q2);

                    d8e = d8ee(i,:);

                    d = [0 -d8e(3) d8e(2)
                         d8e(3) 0 -d8e(1)
                         -d8e(2) d8e(1) 0];

                    D = [eye(3) transpose(d);
                        zeros(3) eye(3)];

%                     matI=[zeros(6,9*6);
%                     zeros(6,1*6), I, I, zeros(6,6*6);
%                     zeros(6,3*6) -I zeros(6,5*6);
%                     zeros(6,4*6) -I zeros(6,4*6);
%                     zeros(6,5*6) -I zeros(6,3*6);
%                     zeros(6,6*6) -I zeros(6,2*6);
%                     zeros(6,9*6);
%                     zeros(6,7*6) I I;
%                     zeros(6,9*6);
%                     zeros(6,7*6) I transpose(D)];
%                 
                    matI=[zeros(6,10*6);
                    zeros(6,1*6), I, I, zeros(6,7*6);
                    zeros(6,3*6) -I zeros(6,6*6);
                    zeros(6,4*6) -I zeros(6,5*6);
                    zeros(6,5*6) -I zeros(6,4*6);
                    zeros(6,6*6) -I zeros(6,3*6);
                    zeros(6,10*6);
                    zeros(6,7*6) I I zeros(6,1*6);
                    zeros(6,10*6);
                    zeros(6,8*6) I transpose(D)];
                    % 
                    %   K_links=[0 I -I 0 0 0 0 0;
                    %            0 0 0 0 0 0 0 0;
                    %            0 0 0 K_11_3 K_12_3 0 0 0 0;
                    %            0 0 0 K_21_3 K_22_3 0 0 0 0;
                    %            0 0 0 0 0 K_11_5 K_12_5 0 0;
                    %            0 0 0 0 0 K_21_5 K_22_5 0 0;
                    %            0 0 0 0 0 0 D -I;
                    %            0 0 0 0 0 0 0 0];
                    %     

                    K_links=[zeros(6,6*1) I -I zeros(6,6*7);
                           zeros(6,6*10);
                           zeros(6,6*3) K_11_3 K_12_3 zeros(6,6*5);
                           zeros(6,6*3) K_21_3 K_22_3 zeros(6,6*5);
                           zeros(6,6*5) K_11_5 K_12_5 zeros(6,6*3);
                           zeros(6,6*5) K_21_5 K_22_5 zeros(6,6*3);
                           zeros(6,6*7) I -I zeros(6,6*1); %for considering a point that connects all end effector
                           zeros(6,6*10);
                           zeros(6,6*8) D -I; %for considering a rigid
%                            platform with specific lenght to center.
                           zeros(6,6*10)];
% 
%                     K_links=[zeros(6,6*1) I -I zeros(6,6*6);
%                            zeros(6,6*9);
%                            zeros(6,6*3) K_11_3 K_12_3 zeros(6,6*4);
%                            zeros(6,6*3) K_21_3 K_22_3 zeros(6,6*4);
%                            zeros(6,6*5) K_11_5 K_12_5 zeros(6,6*2);
%                            zeros(6,6*5) K_21_5 K_22_5 zeros(6,6*2);
%                            zeros(6,6*7) I -I; %for considering a point that connects all end effector
%                            zeros(6,6*9)];

                    lambda_r_12=cell2mat(lambda_r_12_a(i));
                    lambda_e_12=cell2mat(lambda_e_12_a(i));
                    lambda_r_34=cell2mat(lambda_r_34_a(i));
                    lambda_p_34=cell2mat(lambda_p_34_a(i));

                    %  A = [lambda_r_12 -lambda_r_12 0 0 0 0 0 0 0;
                    %       0 0 lambda_r_34 -lambda_r_34 0 0 0 0 0;
                    %       0 0 0 0 lambda_r_34 -lambda_r_34 0 0 0;
                    %       0 0 0 0 0 0 lambda_r_34 -lambda_r_34 0
                    %       I 0 0 0 0 0 0 0 0];
                    %adding the platform:
                    %       0 0 0 0 0 0 D -I];



                    A = [lambda_r_12 -lambda_r_12 zeros(5,6*7);
                          zeros(5,6*2) lambda_r_34 -lambda_r_34 zeros(5,6*5);
                          zeros(5,6*4) lambda_r_34 -lambda_r_34 zeros(5,6*3);
                          zeros(5,6*6) lambda_r_34 -lambda_r_34 zeros(5,6*1);
                          I zeros(6,6*8)];     
                    % 
                    %   B = [I I 0 0 0 0 0 0 0;
                    %       0 0 lambda_r_34 lambda_r_34 0 0 0 0 0;
                    %       0 0 lambda_p_34 0 0 0 0 0 0;
                    %       0 0 0 lambda_p_34 0 0 0 0 0;
                    %       0 0 0 0 lambda_r_34 lambda_r_34 0 0 0;
                    %       0 0 0 0 lambda_p_34 0 0 0 0;
                    %       0 0 0 0 0 lambda_p_34 0 0 0;
                    %       0 0 0 0 0 0 0 lambda_r_34 lambda_r_34 0;
                    %       0 0 0 0 0 0 0 lambda_p_34 0 0;
                    %       0 0 0 0 0 0 lambda_p_34 0]
                    %   

                     B = [I I zeros(6,6*7);
                          zeros(5,6*2) lambda_r_34 lambda_r_34 zeros(5,6*5);
                          zeros(1,6*2) lambda_p_34 zeros(1,6*6);
                          zeros(1,6*3) lambda_p_34 zeros(1,6*5);
                          zeros(5,6*4) lambda_r_34 lambda_r_34 zeros(5,6*3);
                          zeros(1,6*4) lambda_p_34 zeros(1,6*4);
                          zeros(1,6*5) lambda_p_34 zeros(1,6*3);
                          zeros(5,6*6) lambda_r_34 lambda_r_34 zeros(5,6*1);
                          zeros(1,6*6) lambda_p_34 zeros(1,6*2);
                          zeros(1,6*7) lambda_p_34 zeros(1,6*1)];
                    %   
                    C = [lambda_e_12 zeros(1,6*8)];

                    D = [Ka*lambda_e_12 -Ka*lambda_e_12 zeros(1,6*7)];

%                     ABCD=[matI,K_links;
%                           zeros(26,54),A;
%                           B,zeros(27,54);
%                           C,D;
%                           zeros(6,6*8) -I zeros(6,6*9)];
% % 

                    ABCD=[matI,K_links;
                          zeros(26,60),A , zeros(26,6*1);
                          B, zeros(27,6*1),zeros(27,60);
                          C ,zeros(1,6),D ,zeros(1,6);
                          zeros(6,6*9) -I zeros(6,6*10)];
% 
%                     AA=ABCD(1:102,1:102);
%                     BB=ABCD(1:102,103:108);
%                     CC=ABCD(103:108,1:102);
%                     DD=ABCD(103:108,103:108);
                    
                    AA=ABCD(1:114,1:114);
                    BB=ABCD(1:114,115:120);
                    CC=ABCD(115:120,1:114);
                    DD=ABCD(115:120,115:120);



                    Kc = DD-(CC*inv(AA)*BB);
                    rank(Kc);
                    Kc_all=Kc_all+Kc;
                    
                end
                    


                delta_t = inv(Kc_all)*F;
                all_deflections_MSA = [all_deflections_MSA delta_t];
                all_deflections_x(counter) = delta_t(1,1);
                all_deflections_y(counter) = delta_t(2,1);
                all_deflections_z(counter) = delta_t(3,1);
                
                Kc_all = 0;
            end
        end
        
    end
    all_deflections_with_forces_MSA{end+1} = all_deflections_MSA;
    def_mag_MSA = (all_deflections_x.^2 + all_deflections_y.^2 + all_deflections_z.^2).^0.5;

%% Plotting:
  
    figure;
    titles = ['x', 'y', 'z'];
    scatter3(x_all, y_all, z_all,40,def_mag_MSA,'filled')    % draw the scatter plot
    ax = gca;
    ax.XDir = 'reverse';
%     view(-31,14)
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
    title(sprintf(" MSA Method Deflections due to force in %s directions",titles(f)))
    
    colormap(hot);
    cb = colorbar;                                     % create and label the colorbar
    cb.Label.String = 'Deflections Magnitude';
    
%     cb = colorbar;                                     % create and label the colorbar
%     cb.Label.String = 'Deflections Magnitude';
%     figure;
%     quiver3(x_all, y_all, z_all,all_deflections_x,all_deflections_y,all_deflections_z,'LineWidth',1)
%     xlabel("X")
%     ylabel("Y")
%     zlabel("Z")
% %  
end

MSA_time = toc;
