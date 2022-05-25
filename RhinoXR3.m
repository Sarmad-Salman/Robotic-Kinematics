%____________Inverse Kinematics Solution___________
%All units in "degrees" and "millimeters"
%RHINO XR3
d=[26.04 0 0 0 16.83]; %Joint Distance
alpha = [-90 0 0 -90 0];   %Alpha Angle
a=[0 22.86 22.86 0.95 0];   %Link Length
w = [a(3)+a(4) 0 d(1)+a(2)-d(5) 0 0 -0.607];  %Known Final parameters
q1 = atan2d(w(2),w(1));   %Base
q234 = atan2d(-((cosd(q1)*w(4))+(sind(q1)*w(5))), -w(6)); %Coupled Joint
b1 = (cosd(q1)*w(1)) + (sind(q1)*w(2)) - (a(4)*cosd(q234)) + (d(5)*sind(q234));
b2 = d(1) - (a(4)*sind(q234)) - (d(5)*cosd(q234)) - w(3);
bb = b1^2 + b2^2;
q3 = acosd ((bb - a(2)^2 - a(3)^2)/(2*a(2)*a(3))); %Elbow
q2 = atan2d((a(2)+a(3)*cosd(q3))*b2 - a(3)*sind(q3)*b1,...
    (a(2)+a(3)*cosd(q3))*b1 + a(3)*sind(q3)*b2);  %Shoulder
q4 = q234 - q2 - q3;    %Tool Pitch
q5 = 180*log((w(4)^2 + w(5)^2 + w(6)^2)^0.5);   %Tool Roll
q = floor([q1 q2 q3 q4 q5]);
%Link Diagram
L(1) = Link ('revolute','d', d(1),'a', a(1),'alpha', alpha(1));
L(1).qlim =  pi/180*[-180 180]; %Limits of Freedom
L(2) = Link ('revolute','d', d(2),'a', a(2),'alpha', alpha(2));
L(2).qlim =  pi/180*[-180 180];
L(3) = Link ('revolute', 'd',d(3),'a', a(3),'alpha', alpha(3));
L(3).qlim =  pi/180*[-180 180];
L(4) = Link ('revolute', 'd',d(4),'a', a(4),'alpha', alpha(4));
L(4).qlim =  pi/180*[-180 180];
L(5) = Link ('revolute', 'd',d(5),'a', a(5),'alpha', alpha(5));
L(5).qlim =  pi/180*[-180 180];
Rob = SerialLink (L);
Rob.name = 'Rhino XR3'; figure('Name','Home Position');
%Simulation
Rob.plot(q); Rob.teach;