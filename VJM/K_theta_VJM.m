function K_theta = K_theta_VJM(K_active,K_22_3,K_22_5)

    K_theta = [];
%     K_active = 1000000;


    K0 = zeros(1,13);
    K0(1) = K_active;
    K1 = [zeros(6,1) , K_22_3 ,zeros(6,6)];
    K2 = [zeros(6,1) ,zeros(6,6), K_22_5];
    K_theta = [K0 ; K1 ; K2];
    
end