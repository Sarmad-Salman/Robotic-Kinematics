%Trajectory planning for an articulate robot given initial and final
%required angles

% syms a0 a1 a2 a3 t tf theta velocity acc velocityf thetaf
% t = 0; velocity = 0; velocityf = 0;
% eqn = a0 + a1*t + a2*t^2 + a3*t^3==theta;
% eqn2=a1+2*a2*t+3*a3*(t^2)==velocity;
% eqn3=2*a2+6*a3*t==acc;
% A0 = solve (eqn, a0);
% A1 = solve (eqn2, a1);
% eq1= 2*a2*tf+3*a3*(tf^2)==velocityf;
% A3 = solve (eq1,a3);
% eq = A0 + A1*tf + a2*tf^2 + A3*tf^3==thetaf;
% A2 = solve (eq, a2);
syms theta0 thetau thetag tf tf a10 a11 a12 a13 a20 a21 a22 a23
e1 = a10 == theta0;
e2 = a10+a11*tf+a12*tf^2+a13*tf^3 ==thetau;
e3 = a20 == thetau;
e4 = a20+a21*tf+a22*tf^2+a23*tf^3 == thetag;
e5 = a11 ==0;
e6 = a21 +2*a22*tf+3*a23*tf^2 == 0;
e7 =a21 == a11 + 2*a12*tf+3*a12*tf^2 ;
e8 = 2*a22==2*a12+6*a13*tf;
sol = solve ([e1,e2,e3,e4,e5,e6,e7,e8],[a10,a11,a12,a13,a20,a21,a22,a23]);
A10 = sol.a10; A11=sol.a11;  A12=sol.a12; A13=sol.a13;
A20 = sol.a20; A21=sol.a21; A22=sol.a22; A23=sol.a23;