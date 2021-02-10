function q = IK_tri(Tbase, p_global, tr_link, links, leg)

L1 = links(1);
L2 = links(2);

x = p_global(1);
y = p_global(2);
z = p_global(3);

if leg == "X"
    Te = Tx(-tr_link*cosd(30))*Ty(-tr_link*sind(30));
elseif leg == "Y"
    Te = Tx(tr_link*cosd(30))*Ty(-tr_link*sind(30));
elseif leg == "Z"
    Te = Ty(tr_link);
end

P_joint = Tx(x)*Ty(y)*Tz(z)*inv(Te);

P_RR = Tbase(1:3,1:3)'*(P_joint(1:3,4) - Tbase(1:3,4));

RR_x = P_RR(1);
RR_y = P_RR(2);

%% Inverse kinematics

cos_q2 = (RR_x^2 + RR_y^2 - L1^2 - L2^2) / (2*L1*L2);
sin_q2 = sqrt(1 - cos_q2^2);
q2 = atan2(sin_q2, cos_q2);
q1 = atan2(RR_y, RR_x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2));
q3 = - (q1+q2);
q = [q1, q2, q3];

