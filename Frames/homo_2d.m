%SO3 Rotation Matrix Fel 3d - SO bat3amel ma3a matrix 4x4
%SE2 ROtation Matrix Fel 2d - SE bat3amel m3a homogenous matrix mesh 2x2 3x3
clc
clear
grid on;


T1= SE2(1,2,30*pi/180)
trplot2(T1,'frame','1','color','b','arrow');

T2= SE2(2,1,0)
hold on
trplot2(T2,'frame','2','color','r','arrow');


T0=SE2(0,0,0)
hold on
trplot2(T0,'frame','0','color','k','arrow');

T3=T1*T2
hold on
trplot2(T3,'frame','3','color','k','arrow');

T4=T2*T1
hold on
trplot2(T4,'frame','4','color','g','arrow');

p=[3;2]
plot_point(p,'*')

e2h(p) %Vector fel 2d bas fel homo for fa ba2a akeno 3 2 1 ashan a3raf adraboo 

axis([0 5 0 5]);
