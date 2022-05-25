%---------------Inverse Kinematics for Intelledex 660-------------------
%--------------------All units in degrees and mm------------------------
%-------------------Trajectory Planning---------------------------------

d   = [373.4 0 0 0 0 228.6];
a   = [0 0 304.8 304.8 0 0];
alpha = [90 90 0 0 90 0]; %Alpha
w_t = [a(3)+a(4)+d(6) 0 d(1) 1 0 0];
w   = transpose(w_t);T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
R   = [0 0 1; 0 -1 0; 1 0 0];
r_3 = R(:,3);       %This is the last column of R matrix
p   = w(1:3);       %This are the 1st 3 entries of w(q)
i_3 = R(:,1);       %Since this is [0; 0; 1] which is the 1st column of R matrix
q   = double.empty(6,0);
b   = double.empty(3,0);
q6 = 180 * log( (w(4)^2+w(5)^2+w(6)^2)^0.5 );
q2 = -acosd( - R(3,1)*sin(q6) - R(3,2)*cos(q6) );
q1 = atan2d( R(2,1)*sin(q6) + R(2,2)*cos(q6) , R(1,1)*sin(q6) + R(1,2)*cos(q6) );
b    = p - d(6)*r_3 - d(1)*i_3;
b_sq = b(1)^2 + b(2)^2 + b(3)^2;
q4 = acosd( ((abs(b_sq)) - a(3)^2 - a(4)^2) / (2*a(3)*a(4)) );
q3 = atan2d( (a(3)+cos(q4)*a(4))*(sin(q1)*b(1)-cos(q1)*b(2)) - ((sin(q4)*a(4)*b(3))/sin(q2)) ,  sin(q4)*a(4)*(sin(q1)*b(1)-cos(q1)*b(2)) + (((a(3)+cos(q4*a(4))*b(3))/sin(q2))) );
q345 = atan2d( R(3,3), R(3,1)*cos(q6) - R(3,2)*sin(q6) );
q5 = q345 - q3 - q4;
q = round ([q1 q2 q3 q4 q5 q6]); display (q);
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
L(6) = Link ('revolute', 'd',d(6),'a', a(6),'alpha', deg2rad(alpha(6)));
L(6).qlim =  pi/180*[-180 180];
Rob = SerialLink (L);
Rob.name = 'Intellidex 660T Home Position'; figure(1);
%Simulation
Rob.plot(q); Rob.name = 'Intellidex 660T Simulation'; figure(2);
for i = 0:10:90
    for j = 0:10:120
        for k = 30:10:180
            for l = 30:10:180
                for m = 0:10:180
                    for n = 0:10:180
                    Rob.plot([i j k l m n]);
                    q = [i j k l m n]; display(q); %Theeta
                    for o=1:5
                        t = [cosd(q(o)) -cosd(alpha(o))*sind(q(o)) sind(alpha(o))*sind(q(o)) a(o)*cosd(q(o));
                            sind(q(o)) cosd(alpha(o))*cosd(q(o)) -sind(alpha(o))*cosd(q(o)) a(o)*sind(q(o));
                            0 sind(alpha(o)) cosd(alpha(o)) d(o); 0 0 0 1];
                        T = T*t;
                    end
                    p = T(1:3,4); px = p(1); py = p(2); pz = p(3);
                    w = [p;(exp(q(6)/180))*(-sind(q(1))*cosd(q(3)+q(4)+q(5))+cosd(q(1))*...
                        cosd(q(2))*sind(q(3)+q(4)+q(5)));(exp(q(6)/180))*(cosd(q(1))*cosd(q(3)+q(4)+q(5))+sind(q(1))*...
                        cosd(q(2))*sind(q(3)+q(4)+q(5)));(exp(q(6)/180))*sind(q(2))*sind(q(3)+q(4)+q(5))]';
                    display(w);figure(3);plot3(px(:),py(:),pz(:),'.');title('Workspace');
                    ylabel('Y Axis'); xlabel('X Axis');zlabel('Z Axis');grid on; hold on;
                    T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
                    x6 = 180*log((w(4)^2 + w(5)^2 + w(6)^2)^0.5);
                    x2 = -acosd(-R(3,1)*sind(x6) - R(3,2)*cosd(x6));
                    x1 = atan2d(R(2,1)*sind(x6) + R(2,2)*cosd(x6), R(1,1)*sind(x6) + R(1,2)*cosd(x6));
                    I3 = [0 0 1];
                    b = p - d(6)*r_3 - d(1)*I3;
                    bb = b(1)^2 + b(2)^2 + b(3)^2;
                    x345 = atan2d (R(3,3), R(3,1)*cosd(x6)-R(3,2)*sind(x6));
                    x4 = acosd ((bb - a(3)^2 - a(4)^2)/(2*a(3)*a(4)));
                    x3 = atan2d ((a(3)+cosd(x4)*a(4))*(sind(x1)*b(1)-cosd(x1)*b(2))-((sind(x4)*a(4)*b(3))/sind(x2)),...
                        sind(x4)*a(4)*(sind(x1)*b(1)-cosd(x1)*b(2))+(((a(3)+cosd(x4)*a(4))*b(3))/sind(x2)));
                    x5 = x345 - x3 - x4;
                    x = [x1 x2 x3 x4 x5 x6];    %Joint Variables
                    display(x);error = q - x; display(error); figure(4); plot(q,error); grid on;
                    title('Error Plot'); xlabel('Theeta');ylabel('Error');figure(2);
                    end
                end
            end
        end
    end
end