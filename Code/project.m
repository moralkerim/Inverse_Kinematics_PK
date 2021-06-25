clear all; clc

% 
%Generate a Robolink object RDK. This object interfaces with RoboDK.
RDK = Robolink;

% Display the list of all items in the main tree
fprintf('Available items in the station:\n');
disp(RDK.ItemList());

robot = RDK.ItemUserPick('Select one robot', RDK.ITEM_TYPE_ROBOT);

if robot.Valid() == 0
    error('No robot selected');
end

fprintf('Selected robot: %s\n', robot.Name());

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
q2 = [d1       0        a1]';
q3 = [d1       0        a1+a2]';
q4 = [d1+d2    0        a1+a2+a3]';
q5 = q4;
q6 = q4;

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
tetd4=deg2rad(180);
tetd5=deg2rad(61.88);
tetd6=deg2rad(-142.61);

%Desired angles
% tetd1=deg2rad(0);
% tetd2=deg2rad(0);
% tetd3=deg2rad(0);
% tetd4=deg2rad(0);
% tetd5=deg2rad(0);
% tetd6=deg2rad(0);

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
%T1 = TH*inv(M); %Transformation for inserve kinematic calculations.

%Desired Position
Td = real(double(subs(T, [tet1 tet2 tet3 tet4 tet5 tet6], tetd)));

%% Test Forward Kinematic in RoboDK
% pose0 = robot.Pose();
% robot.setPose(Td);

%% Begin I.K Calculations
%Calculate Teta1
% p1 = expm(S1_skew*tetd1)*expm(S2_skew*tetd2)*...
%     expm(S3_skew*tetd3)*hp(q4)
p1 = Td*inv(M)*hp(q4);
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
      0 0 1 0;
      0 0 0 1];

Mp = [1 0 0 d2+d3; 
      0 1 0 0; 
      0 0 1 a1+a2+a3; 
      0 0 0 1];

%Calculate Teta3
T1 = Tt * Td* inv(Mp);
% T1 = Td* inv(M);
p2 = T1 * hp(q4p);

%Verify results
sigma_2 = sqrt(p2(1)^2+p2(2)^2+(p2(3)-a1)^2);
tet0_2 = atan2(a2*d2, -a2*a3);
theta3c_0_2 = acos((d2^2+a3^2+a2^2-p2(1)^2 ...
-p2(2)^2-(p2(3)-a1)^2) / (2*a2*sqrt(d2^2+a3^2)));
thetac3_1_2 = tet0_2 + theta3c_0_2;
thetac3_2_2 = tet0_2 - theta3c_0_2;

tetc3 = thetac3_2_2; %Teta3 value that is being used.

%Solving teta2
v1 = -cross(w1,q1p);
S1_skew = [w1_skew v1; zeros(1,4)];

v2 = -cross(w2,q2p);
S2_skew = [w2_skew v2; zeros(1,4)];

v3 = -cross(w3,q3p);
S3_skew = [w3_skew v3; zeros(1,4)];

v4 = -cross(w4,q4p);
S4_skew = [w4_skew v4; zeros(1,4)];

v5 = -cross(w5,q5p);
S5_skew = [w5_skew v5; zeros(1,4)];

v6 = -cross(w6,q6p);
S6_skew = [w6_skew v6; zeros(1,4)];


 T2 = expm(-S1_skew*tetc1)*  Tt * Td* inv(Mp);

q7_hp = expm(S3_skew*tetc3)*hp(q4p);
p3 = T2*hp(q4p); %p3 hatalı olmalı
p3x = p3(1); p3y = p3(2); p3z = p3(3);
q7x = q7_hp(1); q7y = q7_hp(2); q7z = q7_hp(3);

at1 = p3x*q7z - p3x*a1 - q7x*p3z + q7x*a1;
at2 = q7x*p3x + q7z*p3z - q7z*a1 - p3z*a1 + a1^2;
tetc2 = atan2(at1,at2);

%Solving teta4 and teta5
T3 = expm(-S3_skew*tetc3)*expm(-S2_skew*tetc2)*T2;
q6pp = [0 0 a1+a2+a3]';
p4 = T3*hp(q6pp);

p4x = p4(1); p4y=p4(2); p4z=p4(3);

%Hesaplar manual olarak teyit edildi.
tetc4_1 = atan2( p4y, (a1+a2+a3-p4z));
tetc4_2 = atan2(-p4y,-(a1+a2+a3-p4z));

tetc5_2 = atan2( sqrt(2*p4x*d2-p4x^2),d2-p4x);
tetc5_1 = atan2(-sqrt(2*p4x*d2-p4x^2),d2-p4x);

tetc4 = tetc4_2;
tetc5 = tetc5_2;

%Calculate teta6
T4 = expm(-S5_skew*tetc5)*expm(-S4_skew*tetc4)*T3;
q8 = [0 0 a1+a2]';
p5 = T4 * hp(q8);
p5x = p5(1); p5y = p5(2); p5z = p5(3);
tetc6 = atan2(p5y, p5z-a1-a2-a3);
tetc6 = pi - tetc6;

tetc     = [tetc1 tetc2 tetc3 tetc4 tetc5 tetc6];
tetc_deg = CheckTet(rad2deg(tetc))

robot.setJoints(tetc_deg);