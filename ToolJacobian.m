%Computing Tool Jacobian for a 5 DoF Robot given a home position
%Change the number of angles for a bot of any DoF

syms q1 q2 q3 q4 q5 d4 d1 d5 d3 qq3 d2 q6 w a2 a3 a4 J;
%qq3=deg2rad(180);
w=[];
x=[q1 q2 q3 q4 q5]; %Theta
y=[-90 0 0 -90 0]; %Alpha
d=[d1 0 0 0 d5];
a=[0 a2 a3 a4 0];
T=[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
for i=1:5
    t=[cosd(x(i)*180/pi) -cosd(y(i))*sind(x(i)*180/pi) sind(y(i))*sind(x(i)*180/pi) a(i)*cosd(x(i)*180/pi);
      sind(x(i)*180/pi) cosd(y(i))*cosd(x(i)*180/pi) -sind(y(i))*cosd(x(i)*180/pi) a(i)*sind(x(i)*180/pi);
      0 sind(y(i)) cosd(y(i)) d(i); 0 0 0 1];
    T = T*t;
end
w1=T(1:3,4);
w3=T(1:3,3);
w2=exp(q5/pi)*w3;
w=[w1;w2];
v1=diff(w,q1); v2=diff(w,q2);   v3=diff(w,q3);
v4=diff(w,q4);  v5=diff(w,q5);
J=[v1 v2 v3 v4 v5];
TJ=simplify (J);