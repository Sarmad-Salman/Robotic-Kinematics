%____________Inverse Kinematics Solution___________
%All units in "degrees" and "millimeters"
%SCARA
syms q3; t = 3; alpha = [180 0 0 0]; %Minimum Reach
d=[877 0 q3 200];   %Joint distance
a=[375 375 0 0];    %Link length
w = [0 0 0 0 0 -1]; %Known final parameters
q2 = acosd ((w(1)^2 + w(2)^2 - a(1)^2 - a(2)^2) / (2*a(1)*a(2)));
q1 = atan2d (a(2)*sind(q2)*w(1) + (a(1)+a(2)*cosd(q2))*w(2),...
    (a(1)+a(2)*cosd(q2))*w(1) - a(2)*sind(q2)*w(2));
q3 = d(1) - d(4) - w(3);
q4 = 180*log(abs(w(6)));  q3 = double(q3);
d = [877 0 q3 200];
q = [q1 q2 q3 q4]; %All joint variables
T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]; syms t1;
q_i = [0 0 100 90];     %Initial Position
for i = 1:4
    a0 = q_i(i); a1 = 0;   %Calculated based on the logic that when initial...
    a2 = (3/(t^2)) * (q(i) - q_i(i));     %and final velocities are zero.
    a3 =(-2/(t^3)) * (q(i) - q_i(i));
    p(i) = a0 + a1*t1 + a2*(t1^2) + a3*(t1^3);
    v(i) = a1 + 2.*a2.*t1 + 3.*a3.*(t1.^2);
    ac(i) = 2*a2 + 6*a3*t1; 
end
t1 = 0:0.1:t;   %For the plotting of graphs at different time instances
posmat = zeros(length(t1), 4);
for i = 1:length(t1)
    pos = vpa(subs(p, t1(i)), 2);
    posmat(i, :) = deg2rad(pos);
    posmat(i,3) = pos(3);
    vel = vpa(subs(v, t1(i)), 2);
    velmat(i, :) = vel;
    acc = vpa(subs(ac, t1(i)), 2);
    accmat(i, :) = acc;
    for m=1:4
        t = [cosd(pos(m)) -cosd(alpha(m))*sind(pos(m)) sind(alpha(m))*sind(pos(m)) a(m)*cosd(pos(m));
            sind(pos(m)) cosd(alpha(m))*cosd(pos(m)) -sind(alpha(m))*cosd(pos(m)) a(m)*sind(pos(m));
            0 sind(alpha(m)) cosd(alpha(m)) d(m); 0 0 0 1];
        T = T*t;
    end
    pvec = T(1:3,4); px(:,i) = pvec(1); py(:,i) = pvec(2); pz(:,i) = pvec(3);
    T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
end
%Plot of Position with Time
figure(1); plot (t1, posmat); legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4');
title('Position w.r.t. Time');grid on;xlabel('Time(Seconds)');
ylabel('Position(Degrees)');
%Plot of Velocity with Time
figure(2); plot (t1,velmat);legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4');
legend('show'); title('Velocity w.r.t. Time'); grid on;
xlabel('Time(Seconds)'); ylabel('Velocity(Deg/sec)');
%Plot of Acceleration with Time
figure(3); plot (t1,accmat);legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4');
legend('show'); title('Acceleration w.r.t. Time');grid on;
xlabel('Time(Seconds)'); ylabel('Acceleration(Deg/sec^2)');
%Link Diagram
L(1) = Link('revolute','d', d(1),'a', a(1),'alpha', deg2rad(alpha(1)));
L(1).qlim =  pi/180*[-180 180]; %Limits of Freedom
L(2) = Link('revolute','d', d(2),'a', a(2),'alpha', deg2rad(alpha(2)));
L(2).qlim =  pi/180*[-180 180];
L(3) = Link('prismatic','theta',q_i(3),'a', a(3),'alpha', deg2rad(alpha(3)));
L(3).qlim = [0 q_i(3)];
L(4) = Link('revolute', 'd',d(4),'a', a(4),'alpha', deg2rad(alpha(4)));
L(4).qlim =  pi/180*[-180 180];
Rob = SerialLink (L);
Rob.name = 'Scara Home Position'; figure(4);
Rob.plot(q_i); figure(5);
L(3) = Link('prismatic','theta',q(3),'a', a(3),'alpha', deg2rad(alpha(3)));
L(3).qlim = [0 q(3)];Rob = SerialLink (L);
Rob.name = 'Scara Final Position'; Rob.plot(q); figure(6); Rob.name = 'Scara Simulation';
for i = 1 : length(t1)
    Rob.plot([posmat(i, 1) posmat(i, 2) posmat(i, 3) posmat(i, 4)]);
end
hold on;
scatter3(px, py, pz); xlabel('X axis'); ylabel('Y axis'); zlabel('Z axis');
title('Toolspace Trajectory');