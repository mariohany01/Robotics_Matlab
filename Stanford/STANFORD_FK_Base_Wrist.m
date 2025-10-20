clc;close;clear;

syms theta1 theta2 d3 d1 d2;

%d1=0.25;
%d2=0.15; 

% DH parameters table [theta, d, a, alpha]
DH_table = [theta1,     d1, 0, -pi/2;
            theta2-pi/2, d2, 0,  pi/2;
            -pi/2,       d3, 0,  0];

% Function to compute DH transformation matrix
DH_A = @(theta, d, a, alpha) [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                              sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                              0,           sin(alpha),              cos(alpha),            d;
                              0,           0,                       0,                     1];


T = DH_A(DH_table(1,1), DH_table(1,2), DH_table(1,3), DH_table(1,4));

for ii=2:3
    A=DH_A(DH_table(ii,1),DH_table(ii,2),DH_table(ii,3),DH_table(ii,4));
    T=T*A;
end

simplify(T(1:3,4))



