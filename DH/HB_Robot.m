clc;                                %清屏
clear;                              %清除变量

L(1) = Link([0,  0,     0,   -pi/2], 'standard');
L(2) = Link([0,  0,    1700,   0], 'standard');
L(3) = Link([0,  0,    0,   pi/2], 'standard');
L(4) = Link([0,  1700, 0,   -pi/2], 'standard');
L(5) = Link([0,  0,     0,    pi/2], 'standard');
L(6) = Link([0,  140,   0,    0], 'standard');

% 定义机器人模型
robot = SerialLink([L(1), L(2), L(3), L(4), L(5), L(6)]);

% 显示机器人模型信息
robot.display;

angle = [0,-90,180,0,90,0];
% 绘制机器人模型
robot.plot([0, -pi/2, pi, 0, pi/2, 0]);

% 启动示教模式
robot.teach;


%q=[pi pi/2 pi/2 0 0 0 ];
%T2=[-1,0,0,275;
%     0,1,0,0;                     
%     0,0,-1,450;
%     0,0,0,1;]
%T=fkine(r,q)                   %正解结果，即机器人位姿T(第6关节通过旋转平移与基坐标重合)。
%q1=ikine(r,T2)                  %逆解结果