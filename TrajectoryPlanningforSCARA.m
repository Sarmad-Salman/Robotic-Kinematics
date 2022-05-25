%--------------------Inverse Kinematics for SCARA-----------------------
%--------------------All units in degrees and mm------------------------
%--------------------Workplace trajectory Plotting----------------------

syms q3; d   = [877 0 q3 200];
a   = [375 375 0 0]; T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
%w = [a(1)+a(2) 0 0 0 0 -1];      %Maximum Reach
alpha = [180 0 0 0];               %Minimum Reach
w   = [0 0 0 0 0 -1];
q2 = acosd( (w(1)^2 + w(2)^2 - a(1)^2 - a(2)^2) / (2*a(1)*a(2)) );
q1 = atan2d( a(2)*sin(q2)*w(1) + (a(1)+a(2)*cos(q2))*w(2) , (a(1)+a(2)*cos(q2))*w(1) - a(2)*sin(q2)*w(2));
q3 = d(1) - d(4) - w(3);
q4 =180*log(abs(w(6))); q3 = double(q3);
d   = [877 0 q3 200];
q = floor([q1 q2 q3 q4]);
display (q);
%Link Diagram
% L(1) = Link('revolute','d', d(1),'a', a(1),'alpha', deg2rad(alpha(1)));
% L(1).qlim =  pi/180*[-180 180]; %Limits of Freedom
% L(2) = Link('revolute','d', d(2),'a', a(2),'alpha', deg2rad(alpha(2)));
% L(2).qlim =  pi/180*[-180 180];
% L(3) = Link('prismatic','theta',q(3),'a', a(3),'alpha', deg2rad(alpha(3)));
% L(3).qlim = [0 q3];
% L(4) = Link('revolute', 'd',d(4),'a', a(4),'alpha', deg2rad(alpha(4)));
% L(4).qlim =  pi/180*[-180 180];
% Rob = SerialLink (L);
% Rob.name = 'Scara Home Position'; figure(5);
%Simulation
%Rob.plot(q); Rob.name = 'Scara Simulation'; figure(6);
for i = 0:20:180
    for j = 0:20:100
        for k = 0:20:q3
            for l = 0:20:180
               % Rob.plot([i j k l]);
                q = [i j k l]; %Theeta
                for m=1:4
                    t = [cosd(q(m)) -cosd(alpha(m))*sind(q(m)) sind(alpha(m))*sind(q(m)) a(m)*cosd(q(m));
                        sind(q(m)) cosd(alpha(m))*cosd(q(m)) -sind(alpha(m))*cosd(q(m)) a(m)*sind(q(m));
                        0 sind(alpha(m)) cosd(alpha(m)) d(m); 0 0 0 1];
                    T = T*t;
                end
                p = T(1:3,4); px = p(1); py = p(2); pz = p(3);
                w = [p; 0;0;-exp(q(4)/pi)]';
                figure(7);plot3(px(:),py(:),pz(:),'.');title('Workspace');
                ylabel('Y Axis'); xlabel('X Axis');zlabel('Z Axis');grid on; hold on;
                T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
                x2 = acosd( (w(1)^2 + w(2)^2 - a(1)^2 - a(2)^2) / (2*a(1)*a(2)) );
                x1 = atan2d( abs(a(2)*sin(x2)*w(1) + (a(1)+a(2)*cos(x2))*w(2) ), abs((a(1)+a(2)*cos(x2))*w(1) - a(2)*sin(x2)*w(2)));
                x3 = d(1) - d(4) - w(3);
                x4 =180*log(abs(w(6))); x3 = double(x3);
                d = [877 0 x3 200];x = [x1 x2 x3 x4];
                error = q - x;figure(8); plot(q,error); grid on;
                title('Error Plot'); xlabel('Theeta');ylabel('Error');hold on;figure(6);
            end
        end
    end
end