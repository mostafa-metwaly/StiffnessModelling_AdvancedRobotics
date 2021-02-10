function [q_passive, q_active] = IK_tri(T_base, T_tool, p_global, platform_link, links)

L1 = 1;
L2 = 1;

x = p_global(1);
y = p_global(2);
z = p_global(3);

P_joint = Tx(x)*Ty(y)*Tz(z)*inv(T_tool);

q_active = P_joint(1:3,4);

P_RR = inv(T_base)*Tx(x)*Ty(y)*Tz(z)*inv(T_tool);
P_RR = P_RR(1:3,4);

RR_x = P_RR(1);
RR_y = P_RR(2);

%% Inverse kinematics

cos_q2 = (RR_x^2 + RR_y^2 - L1^2 - L2^2) / (2*L1*L2);
sin_q2 = sqrt(1 - cos_q2^2);
q2 = atan2(sin_q2, cos_q2);
q1 = atan2(RR_y, RR_x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2));
q3 = - (q1+q2);
q_passive = [q1, q2, q3];

