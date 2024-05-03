function Ja = an_Jac(q, pE, TA)

    % joint angles
    q1 = q(1); 
    q2 = q(2); 
    q3 = q(3); 
    q4 = q(4); 
    q5 = q(5); 
    q6 = q(6);
    
    T01 = [cos(q1) 0 -sin(q1) 0; 
           sin(q1) 0 cos(q1) 0; 
           0 -1 0 0.34; 0 0 0 1];
    p1 = T01(1:3,4);

    T12 = [cos(q2) 0 sin(q2) 0; 
           sin(q2) 0 -cos(q2) 0; 
           0 1 0 0; 0 0 0 1];
    T02 = T01*T12;
    p2 = T02(1:3,4); 
    
    T23 = [cos(q3) 0 sin(q3) 0; 
           sin(q3) 0 -cos(q3) 0; 
           0 1 0 0.4; 0 0 0 1];
    T03 = T02*T23;
    p3 = T03(1:3,4);
    
    T34 = [cos(q4) 0 -sin(q4) 0; 
           sin(q4) 0 cos(q4) 0; 
           0 -1 0 0; 0 0 0 1];
    T04 = T03*T34;
    p4 = T04(1:3,4);
    
    T45 = [cos(q5) 0 -sin(q5) 0; 
           sin(q5) 0 cos(q5) 0; 
           0 -1 0 0.4; 0 0 0 1];
    T05 = T04*T45;
    p5 = T05(1:3,4); 
    
    T56 = [cos(q6) 0 sin(q6) 0; 
           sin(q6) 0 -cos(q6) 0; 
           0 1 0 0; 0 0 0 1];
    T06 = T05*T56;
    p6 = T06(1:3,4);
    
    z0 = [0; 0; 1]; 
    z1 = T01(1:3,3);
    z2 = T02(1:3,3);
    z3 = T03(1:3,3); 
    z4 = T04(1:3,3);
    z5 = T05(1:3,3);
    z6 = T06(1:3,3);

    p0 = [0; 0; 0]; 
  
    Jp = [cross(z0,pE-p0) cross(z1,pE-p1) cross(z2,pE-p2) cross(z3,pE -p3) cross(z4,pE-p4) cross(z5,pE-p5) cross(z6,pE-p6)];
    Jo = [z0 z1 z2 z3 z4 z5 z6];
    Jg = [Jp; Jo];
    
    Ja = inv(TA)*Jg;

end