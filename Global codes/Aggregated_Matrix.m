function []=Aggregated_Matrix()

















        ABCD=[matI,K_links;
              zeros(26,60),A , zeros(26,6*1);
              B, zeros(27,6*1),zeros(27,60);
              C ,zeros(1,6),D ,zeros(1,6);
              zeros(6,6*9) -I zeros(6,6*10)];

        AA=ABCD(1:114,1:114);
        BB=ABCD(1:114,115:120);
        CC=ABCD(115:120,1:114);
        DD=ABCD(115:120,115:120);

                    
                    
                    
end