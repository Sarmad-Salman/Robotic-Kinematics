%Computing Transformation Matrix from home position

syms q1 q2 q3 q4 q5 d4 d1 d5 d3 d2 q6 a1 a2 a3 a4 d6;
%q2 = deg2rad(90); q3 = deg2rad(0);
x=[q1 q2 q3 q4 q5 q6]; %Theta
y= [-90 -90 0 90 -90 0]; %Alpha
d=[d1 d2 d3 d4 0 d6];
a=[0 0 0 0 0 0]; 
T=[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

%Iterate the loop for the amount of transformations required
for i=1:2
    t=[cosd(x(i)*180/pi) -cosd(y(i))*sind(x(i)*180/pi) sind(y(i))*sind(x(i)*180/pi) a(i)*cosd(x(i)*180/pi);
      sind(x(i)*180/pi) cosd(y(i))*cosd(x(i)*180/pi) -sind(y(i))*cosd(x(i)*180/pi) a(i)*sind(x(i)*180/pi);
      0 sind(y(i)) cosd(y(i)) d(i); 0 0 0 1];
    T = T*t;
end