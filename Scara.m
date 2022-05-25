%____________Inverse Kinematics Solution___________
%All units in "degrees" and "millimeters"
%SCARA
syms q3; d=[877 0 q3 200];   %Joint distance
a=[375 375 0 0];    %Link length
alpha = [90 0 0 0]; %Alpha angle
w = [0 0 0 0 0 -1]; %Known final parameters
q2 = acosd ((w(1)^2 + w(2)^2 - a(1)^2 - a(2)^2) / (2*a(1)*a(2)));%Elbow
q1 = atan2d (a(2)*sind(q2)*w(1) + (a(1)+a(2)*cosd(q2))*w(2),...
    (a(1)+a(2)*cosd(q2))*w(1) - a(2)*sind(q2)*w(2));    %Base
q3 = d(1) - d(4) - w(3); q3 = double(q3);   %Vertical Extension
q4 = 180*log(abs(w(6)));    %Tool Roll
d=[877 0 q3 200]; q = floor([q1 q2 q3 q4]); %All joint variables
%Link Diagram
L(1) = Link('revolute','d', d(1),'a', a(1),'alpha', alpha(1));
L(1).qlim =  pi/180*[-180 180]; %Limits of Freedom
L(2) = Link('revolute','d', d(2),'a', a(2),'alpha', alpha(2));
L(2).qlim =  pi/180*[-180 180];
L(3) = Link('prismatic','theta',q(3),'a', a(3),'alpha', alpha(3));
L(3).qlim = [0 q3];
L(4) = Link('revolute', 'd',d(4),'a', a(4),'alpha', alpha(4));
L(4).qlim =  pi/180*[-180 180];
Rob = SerialLink (L);
Rob.name = 'Scara'; figure('Name','Home Position');
%Simulation
Rob.plot(q); Rob.teach;