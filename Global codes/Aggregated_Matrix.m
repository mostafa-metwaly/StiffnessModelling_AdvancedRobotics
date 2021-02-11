function Kc = Aggregated_Matrix(jointEq,I)

        jointEq = cell2mat(jointEq)


        ABCD=[jointEq]
        S = size(ABCD)
        AA=ABCD(1:S-6,1:S-6)
        BB=ABCD(1:S-6,S+1-6:S)
        CC=ABCD(S+1-6:S,1:S-6)
        DD=ABCD(S+1-6:S,S+1-6:S)

        Kc = DD-(CC*inv(AA)*BB);

                    
                    
end