function T = trans_mat_7dof(a1,a2,a3,a4,a5,a6,a7,q,alpha)
  
    A01 = [cos(q(1)) 0 sin(q(1))*sin(alpha(1)) 0; 
           sin(q(1)) 0 -cos(q(1))*sin(alpha(1)) 0; 
           0 sin(alpha(1)) cos(alpha(1)) a1; 0 0 0 1];

    A12 = [cos(q(2)) 0 sin(q(2))*sin(alpha(2)) 0; 
           sin(q(2)) 0 -cos(q(2))*sin(alpha(2)) 0; 
           0 sin(alpha(2)) cos(alpha(2)) a2; 0 0 0 1];
    
    A23 = [cos(q(3)) 0 sin(q(3))*sin(alpha(3)) 0; 
           sin(q(3)) 0 -cos(q(3))*sin(alpha(3)) 0; 
           0 sin(alpha(3)) cos(alpha(3)) a3; 0 0 0 1];
        
    A34 = [cos(q(4)) 0 sin(q(4))*sin(alpha(4)) 0; 
           sin(q(4)) 0 -cos(q(4))*sin(alpha(4)) 0; 
           0 sin(alpha(4)) cos(alpha(4)) a4; 0 0 0 1];
        
    A45 = [cos(q(5)) 0 sin(q(5))*sin(alpha(5)) 0; 
           sin(q(5)) 0 -cos(q(5))*sin(alpha(5)) 0; 
           0 sin(alpha(5)) cos(alpha(5)) a5; 0 0 0 1];
       
    A56 = [cos(q(6)) 0 sin(q(6))*sin(alpha(6)) 0; 
           sin(q(6)) 0 -cos(q(6))*sin(alpha(6)) 0; 
           0 sin(alpha(6)) cos(alpha(6)) a6; 0 0 0 1];
       
    A6E = [cos(q(7)) -sin(q(7))*cos(alpha(7)) sin(q(7))*sin(alpha(7)) 0; 
           sin(q(7)) cos(q(7))*cos(alpha(7)) -cos(q(7))*sin(alpha(7)) 0; 
           0 sin(alpha(7)) cos(alpha(7)) a7; 0 0 0 1];
    
    A0E = A01*A12*A23*A34*A45*A56*A6E;
    T = A0E;

end