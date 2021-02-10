function Kc_final = Kc_VJM(K_theta,J_theta,J_q)
    
    Kc_final = zeros(6);
%     i=1;
    for i = 1:length(K_theta)
        J_q_leg = cell2mat(J_q(i));
        J_theta_leg = cell2mat(J_theta(i));
        K_theta_leg = cell2mat(K_theta(i));
        
        Kc_intial = inv(( J_theta_leg * inv(K_theta_leg) * J_theta_leg'));
        Kc = Kc_intial - (Kc_intial* J_q_leg* inv( (J_q_leg' * Kc_intial * J_q_leg) )* J_q_leg' * Kc_intial);
        Kc_final = Kc_final + Kc;

    end
%     Kc_Final = cell2mat(Kc_final(1)) + cell2mat(Kc_final(2)) + cell2mat(Kc_final(3));
end 