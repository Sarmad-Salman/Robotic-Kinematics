%Computing the Manipulator Jacobian Matrix by performing transformations on
%the ARM equation

syms q1 q2 q3 q4 q5 d4 d1 d5 d3 qq3 A B d2 q6 w a2 V FV R2 a3 a4 J FJ tem R1 R P;
syms i1 i2 i3 i4 i5 i6;
%qq3=deg2rad(180);                  %ADD KNOWN ANGLES HERE
w=[]; k=[0; 0; 1];   
R2 = sym(zeros(3, 5));    %in R2 CHANGE THE SECOND ARGUMENT AS NO. OF AXES
x=[q1 q2 q3 q4 q5]; %ADD THE JOINT VARIABLES ACC TO DH
y=[-90 0 0 -90 0]; 
d=[d1 0 0 0 d5];    %REMEMBER TO DECLARE THEM IN START
a=[0 a2 a3 a4 0];
T=[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
for i=1:5               %CHANGE THE LOOPING
    t=[cosd(x(i)*180/pi) -cosd(y(i))*sind(x(i)*180/pi) sind(y(i))*sind(x(i)*180/pi) a(i)*cosd(x(i)*180/pi);
      sind(x(i)*180/pi) cosd(y(i))*cosd(x(i)*180/pi) -sind(y(i))*cosd(x(i)*180/pi) a(i)*sind(x(i)*180/pi);
      0 sind(y(i)) cosd(y(i)) d(i); 0 0 0 1];
        R=T(1:3,1:3);               
    R2(1:3,i)=R*k; 
    T = T*t;
end
w1=T(1:3,4);
w3=T(1:3,3);
%IF ROBOT HAS 4 AXIS REMOVE I5, IF IT HAS 6 AXIS ADD    i6=diff(w1,q6);
i1=diff(w1,q1); i2=diff(w1,q2); i3=diff(w1,q3); i4=diff(w1,q4); i5=diff(w1,q5);
R1=[i1 i2 i3 i4 i5];
A=simplify(R1);
B=simplify(R2);
w2=exp(q5/pi)*w3;       %CHANGE Q5 TO HAVE ROLL ANGLE AKA qn
w=[w1;w2];
%IF ROBOT HAS 4 AXIS REMOVE I5, IF IT HAS 6 AXIS ADD    V6=diff(w1,q6);
v1=diff(w,q1); v2=diff(w,q2);   v3=diff(w,q3);
v4=diff(w,q4);  v5=diff(w,q5);
V=[v1 v2 v3 v4 v5];         %ADD V6 HERE
FV = simplify(V);
J=[A;B];
FJ=simplify(J);
