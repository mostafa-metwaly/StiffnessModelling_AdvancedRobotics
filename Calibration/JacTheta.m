function J = JacTheta(q, thetas, links)

%% Numerical Solution
% Normal forward kinematics model with VJM
% T_FK = Tz(links(1)) * Rz(q(1)) * Rz(thetas(1)) * Tz(q(2)) * Tz(thetas(2)) * Ty(q(3)) * Ty(thetas(3)) * Ty(links(2));
% R_inv = [T_FK(1:3,1:3)' zeros(3,1); 0 0 0 1];
% 
% FK_d1 = Tz(links(1)) * Rz(q(1)) * Rzd(thetas(1)) * Tz(q(2)) * Tz(thetas(2)) * Ty(q(3)) * Ty(thetas(3)) * Ty(links(2)) * R_inv;
% J1 = [FK_d1(1:3,4) ; FK_d1(3,2) ; FK_d1(1,3) ; FK_d1(2,1)];
% 
% FK_d2 = Tz(links(1)) * Rz(q(1)) * Rz(thetas(1)) * Tz(q(2)) * Tzd(thetas(2)) * Ty(q(3)) * Ty(thetas(3)) * Ty(links(2)) * R_inv;
% J2 = [FK_d2(1:3,4) ; FK_d2(3,2) ; FK_d2(1,3) ; FK_d2(2,1)];
% 
% FK_d3 = Tz(links(1)) * Rz(q(1)) * Rz(thetas(1)) * Tz(q(2)) * Tz(thetas(2)) * Ty(q(3)) * Tyd(thetas(3)) * Ty(links(2)) * R_inv;
% J3 = [FK_d3(1:3,4) ; FK_d3(3,2) ; FK_d3(1,3) ; FK_d3(2,1)];
% 
% J = [J1 J2 J3];

%% Analytical Solution:
J1 = [-(links(2) + q(3) + thetas(3) )*cos(q(1) + thetas(1)); -(links(2) + q(3) + thetas(3))*sin(q(1) + thetas(1)); 0; 0; 0; 1];
J2 = [0; 0; 1; 0; 0; 0];
J3 = [-sin(q(1) + thetas(1)); cos(q(1) + thetas(1)); zeros(4,1)];

J = [J1 J2 J3];
