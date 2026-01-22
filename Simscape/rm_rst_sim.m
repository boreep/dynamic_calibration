%% 初始化清理
clc;
clear;
close all;
bdclose('all'); % <--- 【新增】这一行会强制关闭所有已打开的 Simulink 模型，防止重名警告
%%
addpath('URDF/RM65-6FB/urdf/');
ROBOT = importrobot('RM65-6FB.urdf');
showdetails(ROBOT);
% show(ROBOT,'Frames','on','Visuals','on');
ROBOT.Gravity = [0 0 -9.81];

%% add data to workspace
load('dataset_rm65fb/final_traj/traj_N3_200hz_0122.mat');
%% set input variable
qlist = [t' q'];
dqlist = [t' qd'];
ddqlist = [t' q2d'];
dt = dt_new;
%%
config = homeConfiguration(ROBOT);
show(ROBOT)
% config(1).JointPosition = pi/4;
% config(2).JointPosition = pi/6;
show(ROBOT,config);
% load urdf in simulink
% RM65F_SC = smimport('RM65-6FB.urdf');