function x = fwd_kin(a1,a2,a3,a4,a5,a6,a7,q,alpha)
    
    T0E = trans_mat_7dof(a1,a2,a3,a4,a5,a6,a7,q,alpha); % transformation from frame 0 to E
    R0E = T0E(1:3,1:3);
    xe = [T0E(1,4); T0E(2,4); T0E(3,4)];

    phi =  atan2(R0E(2,3),R0E(1,3)); 
    theta = atan2(sqrt((R0E(1,3)^2 + R0E(2,3)^2)), R0E(3,3)); 
    psi = atan2(R0E(3,2), -R0E(3,1));

    x = [xe; phi; theta; psi]; % current position and orientation at q

end