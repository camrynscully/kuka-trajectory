function x = fwd_kin_rect(a1,a2,a3,a4,a5,a6,a7,q,alpha)
    
    A0E = trans_mat_7dof(a1,a2,a3,a4,a5,a6,a7,q,alpha); % transformation from frame 0 to E

    % transformation from frame E to rectangle center
    RE_R = eye(3,3);
    TE_R = [0; 0.0455; 0.06];
    AER = [RE_R TE_R; 0 0 0 1];

    % rotate rectangle frame to match target frame
    RR_R2 = [0 1 0; -1 0 0; 0 0 1]; % aligns x-axis with target's original x-axis
    TR_R2 = [0; 0; 0];
    AR_R2 = [RR_R2 TR_R2; 0 0 0 1];

    T0R2 = A0E*AER*AR_R2;
    xR = [T0R2(1,4); T0R2(2,4); T0R2(3,4)];
    
    phi =  atan2(T0R2(2,3),T0R2(1,3)); 
    theta = atan2(sqrt((T0R2(1,3)^2 + T0R2(2,3)^2)), T0R2(3,3)); 
    psi = atan2(T0R2(3,2), -T0R2(3,1));

    x = [xR; phi; theta; psi]; % current position and orientation at q

end