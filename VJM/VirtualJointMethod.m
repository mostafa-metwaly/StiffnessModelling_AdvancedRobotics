
% ###################################################################################
% codes for Virtual Joint Method

K_22=[E*A/L 0 0 0 0 0;
  0 12*E*Iz/L^3 0 0 0 -6*E*Iz/L^2;
  0 0 12*E*Iy/L^3 0 6*E*Iy/L^2 0;
  0 0 0 G*Ip/L 0 0;
  0 0 6*E*Iy/L^2 0 4*E*Iy/L 0;
  0 -6*E*Iz/L^2 0 0 0 4*E*Iz/L];
K_22_5 = K_22;
K_22_3 = K_22_5;

% K_theta VJM
function K_theta = K_theta_VJM(K_active,K_22_3,K_22_5)

    K_theta = []
%     K_active = 1000000;

    K0 = zeros(1,13);
    K0(1) = K_active;
    K1 = [zeros(6,1) , K_22_3 ,zeros(6,6)];
    K2 = [zeros(6,1) ,zeros(6,6), K_22_5];
    K_theta = [K0 ; K1 ; K2];
    
end



% Kc for the robot:
function Kc_final = Kc_VJM(K_theta,J_theta,J_q)
    Kc_final = [];
    length(K_theta)
    for i = 1:length(K_theta)

        Kc_intial = inv( J_theta(i)*inv(K_theta(i))*transpose(J_theta(i)));
        Kc = Kc_intial - (Kc_intial* J_q(i)* inv(transpose(J_q(i)) * Kc_intial * J_q(i) * transpose(J_q(i), Kc_intial))) ;
        Kc_final = [Kc_final ; Kc ];    

    end
    Kc_Final = Kc_final(1,:) + Kc_final(2,:) + Kc_final(3,:);
end 


%delta T:
function dt = dt_VJM(Kc,F)
dt = inv(Kc)*F;

end

