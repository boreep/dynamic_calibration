%% 1. 加载优化结果
% 请确保这里的文件名是你实际跑出来的那个 mat 文件
load('ptrnSrch_N3T20QR.mat'); 

%% 2. 设置高频插值参数
target_freq = 200; % 目标频率 200Hz
dt_new = 1 / target_freq; 
T = traj_par.T; 
t = 0 : dt_new : T; % 新的时间向量

%% 3. 重新生成轨迹
% 利用保存的参数 c_pol, a, b 进行解析计算，保证绝对光滑
[q, qd, q2d] = mixed_traj(t, c_pol, a, b, traj_par.wf, traj_par.N);

%% 4. 自动构建文件名 (重点在这里)
% 获取阶数 N
N_val = traj_par.N; 

% 获取当前日期，格式化为 'mmdd' (例如 1月22日 -> '0122')
date_str = datestr(now, 'mmdd'); 

% 使用 sprintf 拼接字符串：traj_N{阶数}_200hz_{日期}.csv
filename = sprintf('traj_N%d_%dhz_%s.csv', N_val, target_freq, date_str);

fprintf('准备保存文件: %s ...\n', filename);

%% 5. 保存数据
% 拼接数据矩阵：第一列是时间，后面是关节位置、速度、加速度
% 格式：[t, q1-q6, dq1-dq6, ddq1-ddq6] (共 1+6+6+6 = 19 列)
data_to_save = [t', q', qd', q2d'];

% 保存到当前目录
csvwrite(filename, data_to_save);
% 如果是较新版本的 MATLAB，推荐用 writematrix，精度更高：
% writematrix(data_to_save, filename); 

fprintf('成功保存！文件位于当前目录下。\n');