function T_leg = VJM_T_leg(T_base, T_tool, q_active, thetas, q_passive, links)
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
T_leg = T_base * (Tz(q_active) * Tz(thetas(1)) * Rz(q_passive(1)) * Tx(links(1)) * ( Tx(thetas(2))*Ty(thetas(3))*Tz(thetas(4))*Rx(thetas(5))*Ry(thetas(6))*Rz(thetas(7)) ) * Rz(q_passive(2)) * Tx(links(2)) * ( Tx(thetas(8))*Ty(thetas(9))*Tz(thetas(10))*Rx(thetas(11))*Ry(thetas(12))*Rz(thetas(13)) ) * Rz(q_passive(3)) ) * T_tool;
end