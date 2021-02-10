%% Stiffness Matrix for each link Function
function [K_11f, K_12f, K_21f, K_22f] = StiffnessMatrix(G,E,Length_Links,D,Type)

    if (Type == "rectangle")

        A = D(1)*D(2);
        Iy = (D(1)*D(2)^3)/12;%where base is parallel to the y-axis
        Iz = (D(2)*D(1)^3)/12;%where heigth is parallel to the z-axis
        Ip = Iz+Iy;% Ip is along the x-axis
        
    elseif (Type == "HollowCylinder")
       % Hollow cylinder where D1 is outer and D2 is inner
       A = pi*(D(1)^2-D(2)^2)/4;
       Iy = pi*(D(1)^4-D(2)^4)/64;
       Iz = Iy;
       Ip = Iy+Iz;

    else (Type == "cylinder")
        A = pi*D^2/4;
        Iy = (pi*D^4)/64;
        Iz = (pi*D^4)/64;
        Ip = Iz+Iy;
    end
    
    
    K_11t = [];
    K_12t = [];
    K_21t = [];
    K_22t = [];
    L = 0;
    num = length(Length_Links) ;
    for i = 1 : num 
        L = Length_Links(i)
        
        K_11=[E*A/L 0 0 0 0 0;
          0 12*E*Iz/L^3 0 0 0 6*E*Iz/L^2;
          0 0 12*E*Iy/L^3 0 -6*E*Iy/L^2 0;
          0 0 0 G*Ip/L 0 0;
          0 0 -6*E*Iy/L^2 0 4*E*Iy/L 0;
          0 6*E*Iz/L^2 0 0 0 4*E*Iz/L];

         K_22=[E*A/L 0 0 0 0 0;
          0 12*E*Iz/L^3 0 0 0 -6*E*Iz/L^2;
          0 0 12*E*Iy/L^3 0 6*E*Iy/L^2 0;
          0 0 0 G*Ip/L 0 0;
          0 0 6*E*Iy/L^2 0 4*E*Iy/L 0;
          0 -6*E*Iz/L^2 0 0 0 4*E*Iz/L];

        K_12=[-E*A/L 0 0 0 0 0;
          0 -12*E*Iz/L^3 0 0 0 6*E*Iz/L^2;
          0 0 -12*E*Iy/L^3 0 -6*E*Iy/L^2 0;
          0 0 0 -G*Ip/L 0 0;
          0 0 6*E*Iy/L^2 0 2*E*Iy/L 0;
          0 -6*E*Iz/L^2 0 0 0 2*E*Iz/L];

        K_21=transpose(K_12);
        
        
        K_11t = [K_11t ;K_11];
        K_12t = [K_12t ;K_12];
        K_21t = [K_21t ;K_21];
        K_22t = [K_22t ;K_22];
    
    end


rows_vec = ones(1,num);
K_11f = mat2cell(K_11t,rows_vec*6);
K_12f = mat2cell(K_12t,rows_vec*6);
K_21f = mat2cell(K_21t,rows_vec*6);
K_22f = mat2cell(K_22t,rows_vec*6);


end