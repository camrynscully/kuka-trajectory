%% Camryn Scully – MEEG 671: Final Project 

clear all 
close all

% FWD KINEMATICS - DH parameters 
alpha = [-pi/2 pi/2 pi/2 -pi/2 -pi/2 pi/2 0];

% distances between joints in [m]
d1 = 0.34;
d2 = 0;
d3 = 0.4;
d4 = 0;
d5 = 0.4;
d6 = 0;
d7 = 0.126;
 
% initial configuration of the joints [deg]
q1_deg = [58.2686 75.3224 11.7968 45.9029 -22.1081 -31.2831 -42.3712];
q1 = deg2rad(q1_deg); % [rad]

% configuration of the joints when the picture is taken [deg]
qc_deg = [-77.26 -38.76 26.22 93.29 -56.69 -59.94 118];
qc = deg2rad(qc_deg); % [rad]

% transformation matrix from Frame 0 to Frame E at qc
A0E = trans_mat_7dof(d1,d2,d3,d4,d5,d6,d7,qc,alpha);
R0E = A0E(1:3,1:3);

% transformation from end-effector to camera
REC = [0 1 0; -1 0 0; 0 0 1];
TEC = [0; -0.0662; 0.0431]; % translation WRT end-effector frame
AEC = [REC TEC; 0 0 0 1]; 
A0C = A0E*AEC;

% pose of Aruco marker WRT the camera ref system 
orientA_degZYX = [68.70697922863141 -27.847557028831005 -172.95718336855933]; % z,y,x [deg]
orientA_ZYX = deg2rad(orientA_degZYX); % [rad]

RCA = eul2rotm(orientA_ZYX,'ZYX'); % camera to Aruco marker
TCA = [-0.14195360128424114; -0.06062556383004704; 0.3528046636209403];
ACA = [RCA TCA; 0 0 0 1];
A0A = A0E*AEC*ACA; % base frame to Aruco marker

% Aruco marker to center of target 
RAT = eye(3,3);
TAT = [0.103975; -0.103975; 0];
A_AT = [RAT TAT; 0 0 0 1]; 
A0T = A0E*AEC*ACA*A_AT; % base frame to center of target


% INV KINEMATICS

% end-effector to rectangle origin
RE_R = myrotmat(-pi/2,'z');
TE_R = [0; 0.0455; 0.06];
AE_R = [RE_R TE_R; 0 0 0 1];
A0E_R = A0E*AE_R; 

% desired pose of the target -- matches the rectangle's frame
TT_T2 = DH_mat(0,0,0,pi); % rotate z-axis by 180
T0T2 = A0T*TT_T2;
phiT =  atan2(T0T2(2,3),T0T2(1,3)); 
thetaT = atan2(sqrt((T0T2(1,3)^2 + T0T2(2,3)^2)), T0T2(3,3)); 
psiT = atan2(T0T2(3,2), -T0T2(3,1)); 
PHIT_D = [phiT thetaT psiT]'; % desired orientation of target 
pT = T0T2(1:3,4);

% transformation from geometric to analytical jacobian
TphiT = [0 -sin(phiT) cos(phiT)*sin(thetaT); 0 cos(phiT) sin(phiT)*sin(thetaT); 1 0 cos(thetaT)];
TAT2 = [eye(3,3) zeros(3,3); zeros(3,3) TphiT];

K = 50*eye(6,6);
steps = 100000;
e = zeros(6,steps);
q = zeros(7,steps);
q(:,1) = q1';

limits_deg = [170 120 170 120 170 120 175];
limits = deg2rad(limits_deg); % [rad]

for i = 1:steps
    xr = fwd_kin_rect(d1,d2,d3,d4,d5,d6,d7,q(:,i),alpha);
    Ja = an_Jac(q(:,i),xr(1:3),TAT2);
    e(:,i) = [pT; PHIT_D] - xr;
    qdot = pinv(Ja)*K*e(:,i);
    
    if q(1,i) >= -limits(1) && q(1,i) <= limits(1)
        q(1,i+1) = q(1,i) + qdot(1,1)*0.001;
    else 
        q(1,i+1) = -limits(1);
    end
    if q(2,i) >= -limits(2) && q(2,i) <= limits(2)
        q(2,i+1) = q(2,i) + qdot(2,1)*0.001;
    else
        q(2,i+1) = -limits(2);
    end
    if q(3,i) >= -limits(3) && q(3,i) <= limits(3)
        q(3,i+1) = q(3,i) + qdot(3,1)*0.001;
    else
        q(3,i+1) = -limits(3);
    end
    if q(4,i) >= -limits(4) && q(4,i) <= limits(4)
        q(4,i+1) = q(4,i) + qdot(4,1)*0.001;
    else
        q(4,i+1) = -limits(4);
    end
    if q(5,i) >= -limits(5) && q(5,i) <= limits(5)
        q(5,i+1) = q(5,i) + qdot(5,1)*0.001;
    else
        q(5,i+1) = -limits(5);
    end
    if q(6,i) >= -limits(6) && q(6,i) <= limits(6)
        q(6,i+1) = q(6,i) + qdot(6,1)*0.001;
    else
        q(2,i+1) = -limits(6);
    end
    if q(7,i) >= -limits(7) && q(7,i) <= limits(7)
        q(7,i+1) = q(7,i) + qdot(7,1)*0.001;
    else
        q(7,i+1) = -limits(7);
    end

    if (max(abs(e(:,i))) < 0.000001) 
        break;
    end
end

qf = q(:,i+1);
disp('Joint Angles to move the Rectangle from q1 to the Target:')
disp(q(:,i+1));
xcheck = fwd_kin_rect(d1,d2,d3,d4,d5,d6,d7,qf,alpha);
disp('Pose of Rectangle [WRT Base] given computed joint angles:')
disp(xcheck);
Echeck = fwd_kin(d1,d2,d3,d4,d5,d6,d7,qf,alpha);  
% disp('Pose of End-effector at qf');
% disp(Echeck);

% figure(1)
% plot(e(:,1:i)')
% xlabel('Number of Iterations')
% ylabel('Error')
% legend('x', 'y', 'z', 'phi', 'theta', 'psi')

% JOINT TRAJECTORY
% boundary conditions
vi = 0;
vf = 0;
ai = 0;
af = 0;

tf = 10;
t = 0:0.005:9.995;
qf = q(:,i+1);

for j = 1:7
    A = [tf^5 tf^4 tf^3; 5*(tf)^4 4*(tf)^3 3*(tf)^2; 20*(tf)^3 12*(tf)^2 6*(tf)];
    B = [qf(j) - q1(j); 0; 0];
    C = A\B;

    a0 = q1(j);
    a1 = vi;
    a2 = ai;
    a3 = C(3);
    a4 = C(2);
    a5 = C(1);
    
    joint_pos(:,j) = a5.*t.^5 + a4.*t.^4 + a3.*t.^3 + a2.*t.^2 + a1.*t + q1(j);
    q_dot(:,j) = 5*a5.*t.^4 + 4*a4.*t.^3 + 3*a3.*t.^2 + 2*a2.*t;
    q_ddot(:,j) = 20*a5.*t.^3 + 12*a4.*t.^2 + 6*a3.*t + 2*a2;

    figure(2)
    subplot(3,1,1)
    plot(t,joint_pos)
    hold on
    title('Position')
    xlabel('[s]')
    ylabel('[rad]')

    subplot(3,1,2)
    plot(t,q_dot)
    hold on 
    title('Velocity')
    xlabel('[s]')
    ylabel('[rad/s]')
    
    subplot(3,1,3)
    plot(t,q_ddot)
    title('Acceleration')
    xlabel('[s]')
    ylabel('[rad/s]')
end

% output a trajectory txt file
filename = fopen('scully_camryn.txt','w');
for k = 1:size(joint_pos,1)
    fprintf(filename,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n',joint_pos(k,:));
end
fclose(filename);

% check position and velocity limits
vel_deg = [98 98 100 130 140 180 180];
vel = deg2rad(vel_deg);

poslimit = 0;
vellimit = 0;
for m = 1:length(joint_pos)
    for p = 1:7
        if p==1 
            if joint_pos(m,p) >= limits(1) || joint_pos(m,p) <= -limits(1)
                poslimit = poslimit + 1;
                if q_dot(m,p) >= vel(1)
                    vellimit = vellimit + 1;
                end
            end
        elseif m==2 
            if joint_pos(m,p) >= limits(2) || joint_pos(m,p) <= -limits(2) 
                poslimit = poslimit + 1;
                if q_dot(m,p) >= vel(2)
                    vellimit = vellimit + 1;
                end
            end
        elseif m==3 
            if joint_pos(m,p) >= limits(3) || joint_pos(m,p) <= -limits(3)
                poslimit = poslimit + 1;
                if q_dot(m,p) >= vel(3)
                    vellimit = vellimit + 1;
                end
            end
       elseif m==4
            if joint_pos(m,p) >= limits(4) || joint_pos(m,p) <= -limits(4) 
                poslimit = poslimit + 1;
                if q_dot(m,p) >= vel(4)
                    vellimit = vellimit + 1;
                end
            end
       elseif m==5
            if joint_pos(m,p) >= limits(5) || joint_pos(m,p) <= -limits(5) 
                poslimit = poslimit + 1;
                if q_dot(m,p) >= vel(5)
                    vellimit = vellimit + 1;
                end
            end
       elseif m==6
            if joint_pos(m,p) >= limits(6) || joint_pos(m,p) <= -limits(6) 
                poslimit = poslimit + 1;
                if q_dot(m,p) >= vel(6)
                    vellimit = vellimit + 1;
                end
            end
       elseif m==7
            if joint_pos(m,p) >= limits(7) || joint_pos(m,p) <= -limits(7) 
                poslimit = poslimit + 1;
                if q_dot(m,p) >= vel(7)
                    vellimit = vellimit + 1;
                end
            end
        end
    end
end

if poslimit> 0 
    disp('Joint Position Limit Exceeded!');
    disp(poslimit)
else
    disp('No Position Limits Exceeded');
end

if vellimit> 0 
    disp('Joint Velocity Limit Exceeded!');
    disp(vellimit)
else
    disp('No Velocity Limits Exceeded');
end

% OBSTACLE AVOIDANCE

for q = 1:length(joint_pos)
    xR_pose(:,q) = fwd_kin_rect(d1,d2,d3,d4,d5,d6,d7,joint_pos(q,:),alpha);
    xE_pose(:,q) = fwd_kin(d1,d2,d3,d4,d5,d6,d7,joint_pos(q,:),alpha);
    xR(:,q) = xR_pose(1:3,q);
    xE(:,q) = xE_pose(1:3,q);

    x01 = DH_mat(d1,joint_pos(q,1),0,alpha(1));
    x12 = DH_mat(d2,joint_pos(q,2),0,alpha(2));
    x23 = DH_mat(d3,joint_pos(q,3),0,alpha(3));
    x34 = DH_mat(d4,joint_pos(q,4),0,alpha(4));
    x45 = DH_mat(d5,joint_pos(q,5),0,alpha(5));
    x56 = DH_mat(d6,joint_pos(q,6),0,alpha(6));

    x02 = x01*x12;
    x03 = x01*x12*x23;
    x04 = x01*x12*x23*x34;
    x05 = x01*x12*x23*x34*x45;
    x06 = x01*x12*x23*x34*x45*x56;

    x1(:,q) = x01(1:3,4);
    x2(:,q) = x02(1:3,4);
    x3(:,q) = x03(1:3,4);
    x4(:,q) = x04(1:3,4);
    x5(:,q) = x05(1:3,4);
    x6(:,q) = x06(1:3,4);
end

figure(3)
plot3(xR(1,1),xR(2,1),xR(3,1),'o','MarkerFaceColor', 'g') % initial pos
hold on
plot3(xR(1,2000),xR(2,2000),xR(3,2000),'o','MarkerFaceColor','r') % final pos
hold on 
plot3(xR(1,:),xR(2,:),xR(3,:),'b-') % flange trajectory
hold on 
plot3(xE(1,:),xE(2,:),xE(3,:))
hold on
plot3(x1(1,:),x1(2,:),x1(3,:),'k') % joint 1&2 don't really move
hold on
plot3(x2(1,:),x2(2,:),x2(3,:))
hold on
plot3(x3(1,:),x3(2,:),x3(3,:)) % joint 3&4 are at the same location
hold on
plot3(x4(1,:),x4(2,:),x4(3,:))
hold on
plot3(x5(1,:),x5(2,:),x5(3,:)) % joint 5&6 are at the same location
hold on
plot3(x6(1,:),x6(2,:),x6(3,:))
hold on

% corners of target 
plot3(pT(1)+0.0125,pT(2)+0.0075,pT(3),'o','MarkerFaceColor', 'b')
hold on
plot3(pT(1)-0.0125,pT(2)-0.0075,pT(3),'o','MarkerFaceColor', 'b')
hold on
plot3(pT(1)-0.0125,pT(2)+0.0075,pT(3),'o','MarkerFaceColor', 'b')
hold on
plot3(pT(1)+0.0125,pT(2)-0.0075,pT(3),'o','MarkerFaceColor', 'b')

title('Trajectory')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
legend('Initial Positon','Final Position','Flange','End-Effector','Joint 1','Joint 2', ...
    'Joint 3','Joint 4','Joint 5','Joint 6','Target Corner')
grid on



