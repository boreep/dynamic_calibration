%% 初始化清理
% clc;
% clear;
close all;
bdclose('all'); % <--- 【新增】这一行会强制关闭所有已打开的 Simulink 模型，防止重名警告
%%
addpath('URDF/RM65-6FB/urdf/');
ROBOT = importrobot('RM65-6FB.urdf');
showdetails(ROBOT);
% show(ROBOT,'Frames','on','Visuals','on');
ROBOT.Gravity = [0 0 -9.81];

%% add data to workspace
load('dataset_rm65fb/validation_data/N3_200.mat');
%% set input variable
qlist = [t' q'];
dqlist = [t' dq'];
ddqlist = [t' ddq'];
dt = 0.005;
%%
config = homeConfiguration(ROBOT);
show(ROBOT)
% config(1).JointPosition = pi/4;
% config(2).JointPosition = pi/6;
show(ROBOT,config);
% load urdf in simulink
% RM65F_SC = smimport('RM65-6FB.urdf');

%% 1. 准备 Simulink 需要的动力学参数
%加载辨识结果到工作区
load('RM65_Identified_Result.mat');
% 确保你已经有了 sol 和 baseQR 变量

% 提取刚体基参数
dyn_params.pi_b = sol.pi_b; 

% 提取摩擦力参数
dyn_params.pi_fr = sol.pi_fr;

% 提取降维矩阵 E (从全参数 -> 基参数)
% 这样在 Simulink 里就不用传整个 baseQR 结构体了，只传这个矩阵
dyn_params.E = baseQR.permutationMatrix(:, 1:baseQR.numberOfBaseParameters);

% 判断是否包含电机动力学 (用来决定调用哪个回归函数)
dyn_params.include_motor = 0;
if isfield(baseQR, 'motorDynamicsIncluded')
    dyn_params.include_motor = baseQR.motorDynamicsIncluded;
end

% 将结构体存入工作区，Simulink 稍后会读取这个 'dyn_params'
assignin('base', 'dyn_params', dyn_params);

fprintf('Simulink 动力学参数准备就绪！变量名: dyn_params\n');