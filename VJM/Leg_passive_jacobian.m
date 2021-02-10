function J = Leg_passive_jacobian(T_base, T_tool, q_active, q_passive, thetas, link)
% ** This function calculates the forward kinematics for the given leg ** %

% Structure for each leg (locally):
% 1 - active joint (q_a)
% 2 - 1 dof virtual spring (thetas1) along the active joint axis (local z)
% 3 - passive joint (q_passive1)
% 4 - rigid links1
% 5 - 6 dof virtual spring (thetas2 - thetas7)
% 6 - passive joint (q_passive2)
% 7 - rigid links2
% 8 - 6 dof virtual spring (thetas8 - thetas13)
% 9 - passive joint (q_passive3)

% forward kinematics for each leg
T_leg = T_base * (Tz(q_active) * Tz(thetas(1)) * Rz(q_passive(1)) * Tx(link(1)) * ( Tx(thetas(2))*Ty(thetas(3))*Tz(thetas(4))*Rx(thetas(5))*Ry(thetas(6))*Rz(thetas(7)) ) * Rz(q_passive(2)) * Tx(link(2)) * ( Tx(thetas(8))*Ty(thetas(9))*Tz(thetas(10))*Rx(thetas(11))*Ry(thetas(12))*Rz(thetas(13)) ) * Rz(q_passive(3)) ) * T_tool;
inv_Rot = [T_leg(1:3,1:3)' zeros(3,1); 0 0 0 1]; % inverse of rotation part in forward kinematics

T_J1 = T_base * (Tz(q_active) * Tz(thetas(1)) * Rzd(q_passive(1)) * Tx(link(1)) * ( Tx(thetas(2))*Ty(thetas(3))*Tz(thetas(4))*Rx(thetas(5))*Ry(thetas(6))*Rz(thetas(7)) ) * Rz(q_passive(2)) * Tx(link(2)) * ( Tx(thetas(8))*Ty(thetas(9))*Tz(thetas(10))*Rx(thetas(11))*Ry(thetas(12))*Rz(thetas(13)) ) * Rz(q_passive(3)) ) * T_tool * inv_Rot;
T_J2 = T_base * (Tz(q_active) * Tz(thetas(1)) * Rz(q_passive(1)) * Tx(link(1)) * ( Tx(thetas(2))*Ty(thetas(3))*Tz(thetas(4))*Rx(thetas(5))*Ry(thetas(6))*Rz(thetas(7)) ) * Rzd(q_passive(2)) * Tx(link(2)) * ( Tx(thetas(8))*Ty(thetas(9))*Tz(thetas(10))*Rx(thetas(11))*Ry(thetas(12))*Rz(thetas(13)) ) * Rz(q_passive(3)) ) * T_tool * inv_Rot;
T_J3 = T_base * (Tz(q_active) * Tz(thetas(1)) * Rz(q_passive(1)) * Tx(link(1)) * ( Tx(thetas(2))*Ty(thetas(3))*Tz(thetas(4))*Rx(thetas(5))*Ry(thetas(6))*Rz(thetas(7)) ) * Rz(q_passive(2)) * Tx(link(2)) * ( Tx(thetas(8))*Ty(thetas(9))*Tz(thetas(10))*Rx(thetas(11))*Ry(thetas(12))*Rz(thetas(13)) ) * Rzd(q_passive(3)) ) * T_tool * inv_Rot;

J1 = [T_J1(1,4); T_J1(2,4); T_J1(3,4); T_J1(3,2); T_J1(1,3); T_J1(2,1)];
J2 = [T_J2(1,4); T_J2(2,4); T_J2(3,4); T_J2(3,2); T_J2(1,3); T_J2(2,1)];
J3 = [T_J3(1,4); T_J3(2,4); T_J3(3,4); T_J3(3,2); T_J3(1,3); T_J3(2,1)];

J = [J1 J2 J3];
end