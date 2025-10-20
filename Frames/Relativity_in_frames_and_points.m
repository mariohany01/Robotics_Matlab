clc
clear
close all
grid on;

%base belnesba lel base
base_T_base=SE2(0,0,0)
trplot2(base_T_base,'frame','Base to base','color','k','arrow')

%camera benesba lel base
camera_T_base=SE2(1,2,0)
hold on
trplot2(camera_T_base,'frame','camera to base','color','r','arrow')

%robot benesba lel base
robot_T_base=SE2(3,4,0)
hold on
trplot2(robot_T_base,'frame','robot to base','color','b','arrow')

%base benesba lel robot
base_T_robot=inv(robot_T_base)

%base belnesba lel camera
base_T_camera=inv(camera_T_base)


%point fel graph
base_P=[4;3]
plot_point(base_P,'*');
e2h(base_P)

robot_p=base_T_robot*base_P;
e2h(robot_p)

camera_p=base_T_camera*base_P;
e2h(camera_p)

axis([0 6 0 6]);