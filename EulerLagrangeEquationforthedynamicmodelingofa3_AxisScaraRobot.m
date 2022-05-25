%Dynamic Euler Langrange Equation Modeling for 3 Axis Scara Robot

syms t m1 m2 m3;   % mass
a1=425;  syms a2;         % link distance
syms l3 x y; x=0; y=0;             % prismatic link length
d1 = 877;       % joint distance
q1 = 0; q2 =180;  q3=677;     % joint angles
syms J1 v1 v2 v3 ac1 ac2 ac3                % link jacobian, velocity, acc.
syms bv1 bd1 bs1 bv2 bd2 bs2 bv3 bd3 bs3    % friction coefficients (links)
limitedTrans = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];  L = 3;
q = [q1 q2 q3]; d = [d1 0 q3]; a = [a1 a2 0]; alp = [pi 0 0];
dh = [q; alp; d; a]; h =0; m = [m1 m2 m3];         % mass of links in vector form
g = [0; 0; -9.8062];    % gravity loading vector for SCARA
b = [bv1 bd1 bs1;       % friction coefficients in matrix form
    bv2 bd2 bs2;
    bv3 bd3 bs3]; vel = [v1 v2 v3]; acc = [ac1 ac2 ac3];
Dq = sym(zeros(3, L)); A_Store = sym(zeros(3*L, L));
for i = 1 : L
    if i < 3
        c = [-a(i)/2; 0; 0; 1];  diag = eye(3);
        diag(1, 1) = 0;   D_bar = ((m(i)*a(i)^2)/12)*diag;
    else
        c = [0; 0; -l3/2; 1];  diag = eye(3);
        diag(3, 3) = 0;  D_bar = ((m(i)*l3^2)/12)*diag;
    end
    for k = 1 : i
        arm_eq = [cos(dh (1, k)) -cos(dh (2, k))*sin(dh (1, k))   sin(dh (2, k))*sin(dh (1, k)) dh(4, k)*cos(dh (1, k));
            sin(dh (1, k))  cos(dh (2, k))*cos(dh (1, k))  -sin(dh (2, k))*cos(dh (1, k)) dh(4, k)*sin(dh (1, k));
            0               sin(dh (2, k))                  cos(dh (2, k))                dh(3, k);
            0                              0                               0               1                    ];
        T = T * arm_eq;
    end
    H = [eye(3) zeros(3, 1)]; c_bar = H*T*c;
    R = T(1:3, 1:3);  D = R*D_bar*transpose(R);
    j = sym(zeros(6,3)); etta = [1 1 0];
    % viscous, dynamic, static
    % Calculating each column of B (bottom 3x3) of the Jacobian Manipulator
    for n = 1 : i
        for k = 1 : n-1
            arm_eq = [cos(dh (1, k)) -cos(dh (2, k))*sin(dh (1, k))   sin(dh (2, k))*sin(dh (1, k)) dh(4, k)*cos(dh (1, k));
                sin(dh (1, k))  cos(dh (2, k))*cos(dh (1, k))  -sin(dh (2, k))*cos(dh (1, k)) dh(4, k)*sin(dh (1, k));
                0               sin(dh (2, k))                  cos(dh (2, k))                dh(3, k);
                0                              0                               0               1                    ];
            limitedTrans = limitedTrans * arm_eq;
        end
        bee = etta(n)*limitedTrans(1:3, 3); j (4:6, n) = bee;
    end
    %calculating each column of A (top 3x3) of the Jacobian Manipulator
    for n = 1 : L
        j (1:3, n) = diff(c_bar, q(n));
    end
    j = simplify(j);
    A = j(1:3, :); B = j(4:6, :);   % partitions J into two parts of 3x3
    %(L*i)-2 : (L*i) gives rows 1 to 3 (1st run), then 4 to 6, then 7 to 9
    A_Store((L*i)-2 : (L*i), :) = A;
    Dq = Dq + transpose(A)*m(i)*A + transpose(B)*D*B;
end
Dq = simplify(Dq); C_Store = sym(zeros(3*L, L));
h_Store = sym(zeros(1, 3)); model = sym(zeros(3, 1));
syms e % a small positive parameter, 'epsilon' in book
bk = sym(zeros(1, L));
for k = 1 : L
    bk(1, k) = b(k, 1) + sign(vel(k))*(b(k, 2) + (b(k, 3) - ...
        b(k, 2))*exp(-abs(vel(k))/e));
end
simplify(bk);
for i= 1 : L
    C = sym(zeros(L, L));
    for j = 1 : L
        for k = 1 : L
            C(k, j) = diff(Dq(i, j), q(k)) - 0.5*diff(Dq(k, j), q(i));
        end
    end
    C = simplify(C); C_Store( (L*i)-2 : (L*i), : ) = C;
    for k = 1 : L
        for j = i : L
            h = h + g(k)*m(j)*A_Store(j*L - (L-k), L);
        end
    end
    h = -h; h_Store(1, i) = h;
    for k = 1 : L
        x = x + Dq(i, k)*acc(k);
        for j = 1 : L
            y = y + C(k, j)*vel(k)*vel(j);
        end
    end
    transfomed = x + y + h + bk(i);  transfomed = simplify(transfomed);  model(i, 1) = simplify(transfomed);
end
model = simplify(model); disp('Model = '); disp(model);