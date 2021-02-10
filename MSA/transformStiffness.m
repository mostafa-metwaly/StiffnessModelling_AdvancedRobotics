function [Q1 , Q2] = transformStiffness(T_base, p_global, q_passive, link)
Q1 = [];
Q2 = [];
    for i = 1:size(T_base,3)
        q = q_passive(i,:);
        toOrigin = T_base(:,:,i);
%         origin = toOrigin[0:3, 3]

        toLink1 = toOrigin*Tz(p_global(i))*Rz(q(1));
        rotationLink1 = toLink1(1:3, 1:3);

        toLink2 = toLink1*Tx(link(1))*Rz(q(2));
        rotationLink2 = toLink2(1:3, 1:3);

        zeros_3 = zeros(3,3);

        Q11 = [rotationLink1,         zeros_3;
              zeros_3,         rotationLink1];

        Q22 = [rotationLink2,         zeros_3;
              zeros_3,         rotationLink2];
          
        if i == 1
            Q1(:,:,size(Q1,3)) = Q11;
            Q2(:,:,size(Q2,3)) = Q22;
        else
            Q1(:,:,size(Q1,3)+1) = Q11;
            Q2(:,:,size(Q2,3)+1) = Q22;
        end
%         Q(:,:,size(Q,3)+1) = Q2;
    end