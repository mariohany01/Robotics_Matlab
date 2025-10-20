clc
clear
close all
grid on;

%base belnesba lel base
base_T_base=SE2(0,0,0)
trplot2(base_T_base,'frame','Base to base','color','k','arrow')


%camera benesba lel base
robot_T_base=SE2(3,4,pi)
hold on
trplot2(robot_T_base,'frame','camera to base','color','b','arrow')


camera_T_base=SE2(1,2,30*pi/180)
hold on
trplot2(camera_T_base,'frame','camera to base','color','r','arrow')


base_T_robot=inv(robot_T_base)

cam_T_robot=base_T_robot*camera_T_base

axis([0 5 0 5])