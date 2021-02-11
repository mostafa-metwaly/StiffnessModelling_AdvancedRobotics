
 function LinkEqT = Link_Equations(AllNodes,descriptionLink,G,E,Links ,D,LinkType)

I = eye(6);
i = 0;
LinkEqT = [];
LinkEq = [];
numnodes = length(AllNodes);
 
% Stiffness Matrix for each link
[K_11a,K_12a,K_21a,K_22a] = StiffnessMatrix(G,E,Links ,D,LinkType);

 
    d8e = [0.1 0.2 0.4];
    d = [0 -d8e(3) d8e(2)
         d8e(3) 0 -d8e(1)
         -d8e(2) d8e(1) 0];
    D = [eye(3) transpose(d);
        zeros(3) eye(3)];
    

    
for a =1 : numnodes
    if (AllNodes(a) ~= 1)
        i = i+ 1
        node = i; nodef = i+1;
        

        K_11 = cell2mat(K_11a(i));
        K_12 = cell2mat(K_12a(i));
        K_21 = cell2mat(K_21a(i));
        K_22 = cell2mat(K_22a(i));

        
        
    %% Links:
        %% Rigid Links
        if (descriptionLink(i) == "RigidLinks")

            LinkEq = mat2cell(zeros(12,2*numnodes*6),[6 6],ones(1,numnodes*2)*6);
            LinkEq(1,node+numnodes) = mat2cell(D,[6],[6]);
            LinkEq(1,nodef+numnodes) = mat2cell(-I,[6],[6]);
            LinkEq(2,node) = mat2cell(I,[6],[6]);
            LinkEq(2,nodef) = mat2cell(D',[6],[6]);




        %% Flexible links:
        elseif (descriptionLink(i) == "FlexibleLinks")

            LinkEq = mat2cell(zeros(12,2*numnodes*6),[6 6],ones(1,numnodes*2)*6);
            LinkEq(1,node) = mat2cell(-I,[6],[6]);
            LinkEq(2,nodef) = mat2cell(-I,[6],[6]);

            LinkEq(1,node+numnodes) = mat2cell(K_11,[6],[6]);
            LinkEq(1,nodef+numnodes) = mat2cell(K_12,[6],[6]);
            LinkEq(2,node+numnodes) = mat2cell(K_21,[6],[6]);
            LinkEq(2,nodef+numnodes) = mat2cell(K_22,[6],[6]);

        end

        LinkEqT = [LinkEqT ;LinkEq];

    end
 end