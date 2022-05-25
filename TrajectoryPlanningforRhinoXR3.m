%------------------Inverse Kinematics for Rhino R3----------------------
%--------------------All units in degrees and mm------------------------
%-------------------Trajectory Planning---------------------------------

d = [26.04 0 0 0 16.83];
a = [0 22.86 22.86 0.95 0];
alpha = [-90 0 0 -90 0];
T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
w = [a(3)+a(4) 0 d(1)+a(2)-d(5) 0 0 -0.607];
q(1) = atan2d(w(2),w(1));
q234 = atan2d( -((cosd(q(1)) * w(4)) + (sind(q(1)) * w(5))) , -w(6));
b1   = (cosd(q(1))*w(1)) + (sind(q(1))*w(2)) - (a(4)*cosd(q234)) + (d(5)*sind(q234));
b2   = d(1) - (a(4) * sind(q234)) - (d(5) * cosd(q234)) - w(3);
b_sq = power(b1,2) + power(b2,2);
temp = (b_sq - a(2)^2 - a(3)^2) / (2 * a(2) * a(3));
q(3) = acosd(temp);
q(2) = atan2d( (a(2)+a(3)*cosd(q(3)))*b2 - a(3)*sind(q(3))*b1, (a(2)+a(3)*cosd(q(3)))*b1 + a(3)*sind(q(3))*b2);
q(4) = q234 - q(2) - q(3);
q(5)=180*log( (w(4)^2 + w(5)^2 + w(6)^2)^0.5 );
q = floor (q); display (q);
%Link Diagram
L(1) = Link ('revolute','d', d(1),'a', a(1),'alpha', deg2rad(alpha(1)));
L(1).qlim =  pi/180*[-90 90]; %Limits of Freedom
L(2) = Link ('revolute','d', d(2),'a', a(2),'alpha', deg2rad(alpha(2)));
L(2).qlim =  pi/180*[0 120];
L(3) = Link ('revolute', 'd',d(3),'a', a(3),'alpha', deg2rad(alpha(3)));
L(3).qlim =  pi/180*[30 180];
L(4) = Link ('revolute', 'd',d(4),'a', a(4),'alpha', deg2rad(alpha(4)));
L(4).qlim =  pi/180*[30 180];
L(5) = Link ('revolute', 'd',d(5),'a', a(5),'alpha', deg2rad(alpha(5)));
L(5).qlim =  pi/180*[-180 180];
Rob = SerialLink (L);
Rob.name = 'Rhino XR3 Home Position'; figure(1);
%Simulation
%Rob.plot(q); Rob.name = 'Rhino XR3 Simulation'; figure(2);
for i = -90:20:90
    for j = 0:20:120
        for k = 30:20:180
            for l = 30:20:180
                for m = -180:20:180
                    %Rob.plot([i j k l m]);
                    q = [i j k l m]; display(q); %Theeta
                    for n=1:5
                        t = [cosd(q(n)) -cosd(alpha(n))*sind(q(n)) sind(alpha(n))*sind(q(n)) a(n)*cosd(q(n));
                            sind(q(n)) cosd(alpha(n))*cosd(q(n)) -sind(alpha(n))*cosd(q(n)) a(n)*sind(q(n));
                            0 sind(alpha(n)) cosd(alpha(n)) d(n); 0 0 0 1];
                        T = T*t;
                    end
                    p = T(1:3,4); px = p(1); py = p(2); pz = p(3);
                    w = [p;-(exp(q(5)/180))*cosd(q(1))*sind(q(2)+q(3)+q(4));-(exp(q(5)/180))...
                        *sind(q(1))*sind(q(2)+q(3)+q(4));-(exp(q(5)/180))*cosd(q(2)+q(3)+q(4))]';
                    display(w);figure(3);plot3(px(:),py(:),pz(:),'.');title('Workspace');
                    ylabel('Y Axis'); xlabel('X Axis');zlabel('Z Axis');grid on; hold on;
                    T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
                    x(1) = atan2d(w(2),w(1));
                    x234 = atan2d( -((cosd(x(1)) * w(4)) + (sind(x(1)) * w(5))) , -w(6));
                    b1   = (cosd(x(1))*w(1)) + (sind(x(1))*w(2)) - (a(4)*cosd(x234)) + (d(5)*sind(x234));
                    b2   = d(1) - (a(4) * sind(x234)) - (d(5) * cosd(x234)) - w(3);
                    b_sq = power(b1,2) + power(b2,2);
                    temp = (b_sq - a(2)^2 - a(3)^2) / (2 * a(2) * a(3));
                    x(3) = acosd(temp);
                    x(2) = atan2d( (a(2)+a(3)*cosd(x(3)))*b2 - a(3)*sind(x(3))*b1, (a(2)+a(3)*cosd(x(3)))*b1 + a(3)*sind(x(3))*b2);
                    x(4) = x234 - x(2) - x(3);
                    x(5)=180*log( (w(4)^2 + w(5)^2 + w(6)^2)^0.5 );
                    display (x); error = q - x; display(error); figure(4); plot(q,error); grid on;
                    title('Error Plot'); xlabel('Theeta');ylabel('Error');hold on;figure(2);
                end
            end
        end
    end
end