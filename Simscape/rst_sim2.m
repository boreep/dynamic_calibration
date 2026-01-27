%% 初始化清理
% clc;
% clear;
close all;
bdclose('all'); % <--- 【新增】这一行会强制关闭所有已打开的 Simulink 模型，防止重名警告
%%
addpath('URDF/RM65-6FB/urdf/');
ROBOT = importrobot('left_robot.urdf');
showdetails(ROBOT);
% show(ROBOT,'Frames','on','Visuals','on');
% ROBOT.Gravity = [0 0 -9.81];
ROBOT.Gravity =[9.81 0 0];

%% 1. 读取 CSV 数据 (替换 load .mat)
% 确保文件路径正确，这里使用相对路径
csv_filename = 'my_test/q_ga_5.csv'; 
raw_data = readmatrix(csv_filename); % 读取数据，假设为 N行6列

%% 2. 设置变量 (Set Input Variable)
dt = 0.005; % 时间步长 (5ms)
[N, dim] = size(raw_data); % N: 采样点数, dim: 关节数(6)

% 生成时间向量 t (行向量 1xN，与原始代码逻辑保持一致)
t = (0:N-1) * dt; 

% 提取位置 q
% 原始代码中 qlist = [t' q']，说明 q' 是 (Nx6)，即 q 是 (6xN)
% 我们的 raw_data 是 (Nx6)，所以对应 q'
q_transpose = raw_data; % (Nx6)
q = q_transpose';       % (6xN)

% 计算速度 dq 和 加速度 ddq
% 由于 CSV 只有位置，我们需要通过数值微分计算速度和加速度
% 使用 gradient 函数进行中心差分 (保持数据长度不变)
dq = zeros(dim, N);
ddq = zeros(dim, N);

for i = 1:dim
    dq(i, :) = gradient(q(i, :), dt);    % 计算一阶导数 (速度)
    ddq(i, :) = gradient(dq(i, :), dt);  % 计算二阶导数 (加速度)
end

%% 3. 构造输出列表 (保持原始格式)
% qlist, dqlist, ddqlist 均为 Nx7 矩阵 (第一列是时间)
qlist = [t' q'];     
dqlist = [t' dq'];
ddqlist = [t' ddq'];
dt = 0.005;

% 检查一下数据
fprintf('数据加载完成:\n');
fprintf('点数 (N): %d\n', N);
fprintf('时长: %.2f 秒\n', t(end));



%% add data to workspace
% load('dataset_rm65fb/validation_data/N3_200.mat');
% %% set input variable
% qlist = [t' q'];
% dqlist = [t' dq'];
% ddqlist = [t' ddq'];
% dt = 0.005;
%%
config = homeConfiguration(ROBOT);
show(ROBOT)
show(ROBOT,config);
% load urdf in simulink
RM65F_SC = smimport('left_robot.urdf');
