function [Kc ,S] = Aggregated_Matrix(jointEq,LinkEq)
        

        EquationT = [jointEq;LinkEq];
        ABCD = cell2mat(EquationT);


        S = size(ABCD)
        AA=ABCD(1:S-6,1:S-6);
        BB=ABCD(1:S-6,S-5:S);
        CC=ABCD(S-5:S,1:S-6);
        DD=ABCD(S-5:S,S-5:S);
        
        

        Kc = DD-(CC*inv(AA)*BB);

                    Kc_all=Kc_all+Kc;
                    
                end
                    


                delta_t = inv(Kc_all)*F;
                all_deflections_MSA = [all_deflections_MSA delta_t];
                all_deflections_x(counter) = delta_t(1,1);
                all_deflections_y(counter) = delta_t(2,1);
                all_deflections_z(counter) = delta_t(3,1);
                
    all_deflections_with_forces_MSA{end+1} = all_deflections_MSA;
    def_mag_MSA = (all_deflections_x.^2 + all_deflections_y.^2 + all_deflections_z.^2).^0.5;

                    
                    
end