clear all; clc

% 
% Generate a Robolink object RDK. This object interfaces with RoboDK.
% RDK = Robolink;
% 
% % Display the list of all items in the main tree
% fprintf('Available items in the station:\n');
% disp(RDK.ItemList());
% 
% robot = RDK.ItemUserPick('Select one robot', RDK.ITEM_TYPE_ROBOT);
% 
% if robot.Valid() == 0
%     error('No robot selected');
% end
% 
% fprintf('Selected robot: %s\n', robot.Name());
% 
%% Calculate Forward Kinematics

%Robot parameters
a1 = 585; a2 = 650; a3 = 192;
d1 = 190; d2 = 730; d3 = 132;

%Angle variables
syms tet1 tet2 tet3 tet4 tet5 tet6

%Screw axes
w1 = [0 0 1]';
w2 = [0 1 0]';
w3 = [0 1 0]';
w4 = [1 0 0]';
w5 = [0 1 0]';
w6 = [1 0 0]';

%Screw points
q1 = [0        0        0]';
q2 = [d1        0        a1]';
q3 = [d1        0        a1+a2]';
q4 = [d1+d2       0        a1+a2+a3]';
q5 = [d1+d2    0        a1+a2+a3]';
q6 = [d1+d2 0        a1+a2+a3]';

%Point velocities
v1 = -cross(w1,q1);
v2 = -cross(w2,q2);
v3 = -cross(w3,q3);
v4 = -cross(w4,q4);
v5 = -cross(w5,q5);
v6 = -cross(w6,q6);

%Home angles
tet0 = [0 0 0 0 0 0];

%Desired angles
tetd1=deg2rad(-4.57);
tetd2=deg2rad(8.88);
tetd3=deg2rad(17.94);
tetd4=deg2rad(0);
tetd5=deg2rad(61.88);
tetd6=deg2rad(37.39);

tetd = [tetd1 tetd2 tetd3 tetd4 tetd5 tetd6];

%Skews
w1_skew = skew(w1);
S1_skew = [w1_skew v1; zeros(1,4)];

w2_skew = skew(w2);
S2_skew = [w2_skew v2; zeros(1,4)];

w3_skew = skew(w3);
S3_skew = [w3_skew v3; zeros(1,4)];

w4_skew = skew(w4);
S4_skew = [w4_skew v4; zeros(1,4)];

w5_skew = skew(w5);
S5_skew = [w5_skew v5; zeros(1,4)];

w6_skew = skew(w6);
S6_skew = [w6_skew v6; zeros(1,4)];

%Home position
R0 = [1 0 0; 0 1 0; 0 0 1];
q0 = [d1+d2+d3 0        a1+a2+a3]';
M = [R0 q0; 0 0 0 1];

%Compute forward kinematics
T = expm(S1_skew * tet1);
T = T * expm(S2_skew * tet2);
T = T * expm(S3_skew * tet3);
T = T * expm(S4_skew * tet4) ;
T = T * expm(S5_skew * tet5) ;
T = T * expm(S6_skew * tet6) ;

T = T * M;

%Home Position
TH = double(subs(T, [tet1 tet2 tet3 tet4 tet5 tet6], tet0));
T1 = TH*inv(M); %Transformation for inserve kinematic calculations.

%Desired Position
Td = real(double(subs(T, [tet1 tet2 tet3 tet4 tet5 tet6], tetd)));

%% Test Forward Kinematic in RoboDK
% pose0 = robot.Pose();
% robot.setPose(Td);

%% Begin I.K Calculations
%Calculate Teta1
p1 = expm(S1_skew*tetd1)*expm(S2_skew*tetd2)*...
    expm(S3_skew*tetd3)*hp(q4)
p1 = Td*inv(M)*hp(q4)
p1x = p1(1); p1y = p1(2);

tetc1_1 = atan2( p1y, p1x);
tetc1_2 = atan2(-p1y,-p1x);

tetc1 = tetc1_1; %Use first or second teta1 value

%Translated points for non-singularity
q1p = [0 0 0]';      q2p = [0 0 a1]';
q3p = [0 0 a1+a2]';  q4p = [d2 0 a1+a2+a3]';
q5p = q4p;           q6p = q4p;

Tt = [1 0 0 -d1*cos(tetc1);
      0 1 0 -d1*sin(tetc1);
      0 0 1 0
      0 0 0 1];

Mp = [1 0 0 d2+d3; 
      0 1 0 0; 
      0 0 1 a1+a2+a3; 
      0 0 0 1]
  
%Calculate Teta3
p2 = Tt * Td* inv(Mp) * hp(q4p) %%**p2 hesabı yanlış
p2 = expm(S1_skew * tetd1)*expm(S2_skew * tetd2)...
   * expm(S3_skew * tetd3) * hp(q4p)

r3 = [0 0 a1+a2]';
sigma = norm((p2)-hp(q2p))
[tet0, tet3_0] = pk3(w3,q4p,q2,r3,sigma);
theta3c_1 = tet0+tet3_0
theta3c_2 = tet0-tet3_0

%Verify results
tet0_2 = atan2(a2*d2, -a2*a3);
theta3c_0_2 = acos((d2^2+a3^2+a2^2-p2(1)^2 ...
-p2(2)^2-(p2(3)-a1)^2) / (2*a2*sqrt(d2^2+a3^2)));
theta3c_1_2 = tet0_2 + theta3c_0_2
theta3c_2_2 = tet0_2 - theta3c_0_2



