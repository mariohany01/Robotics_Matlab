clc 
clear
close all
grid on
 
%transl (1,2,3)
%rotx(180)
%trotx(180)
%transl (1,2,3)*trotx(180)   %rotation then trnaslation 
                            %El homog transformation matrix heya 3amaleyet
                            %rotation ba3d keda 3amaleyet transformation

Base=eye(4)
trplot(Base,'frame','0','color','k','arrow')


T01=transl(0,0,3)*troty(90)*trotx(90)
hold on
trplot(T01,'frame','1','color','k','arrow')

axis([-1 5 -1 5 -1 5])

T02=transl(0,3,0)*troty(-90)*trotz(-90)
hold on
trplot(T02,'frame','2','color','k','arrow')
                            
%T21=T20*T01
%but i dont have T20
%so ill calculate it by inverse

T21=inv(T02)*T01

P0=[0;0;0]
e2h(P0)

%3ayez ageb el point dei belnesba le frame 2
T20=inv(T02)

P2=T20*e2h(P0)


tranimate(T01)
tranimate(T02)
tranimate(T12)




