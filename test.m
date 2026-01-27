%% 基于辨识参数预测力矩 (修复数据提取版)
%加载辨识结果到工作区
load('RM65_Identified_Result.mat');
%加载仿真运行结果out
load('Simscape/sim_output/old_N5_0122.mat')

% =========================================================================
% 1. 智能提取数据 (兼容 Matrix, Timeseries, Struct)
% =========================================================================

% 定义一个通用提取函数 (Local Function logic)
extract_data = @(val) get_payload(val);

% --- A. 获取时间 ---
if isprop(out, 'tout') || isfield(out, 'tout')
    t = out.tout;
elseif isprop(out, 't') || isfield(out, 't')
    t = out.t;
else
    % 尝试从 q 里面拿时间
    temp_q = extract_signal_raw(out, 'q');
    if isa(temp_q, 'timeseries')
        t = temp_q.Time;
    else
        error('无法找到时间向量 (tout)');
    end
end

% --- B. 获取信号数据 ---
q_list   = extract_data(extract_signal_raw(out, 'q'));
dq_list  = extract_data(extract_signal_raw(out, 'dq'));
ddq_list = extract_data(extract_signal_raw(out, 'ddq'));
tau_meas = extract_data(extract_signal_raw(out, 'tor'));

N = length(t);

% =========================================================================
% 2. 准备参数
% =========================================================================
pi_total = [sol.pi_b; sol.pi_fr]; 

% =========================================================================
% 3. 核心计算循环
% =========================================================================
tau_pred = zeros(N, 6);

fprintf('正在计算力矩 (N=%d)...\n', N);
for i = 1:N
    qi = q_list(i, :)';
    dqi = dq_list(i, :)';
    ddqi = ddq_list(i, :)';
    
    % --- 判断是否包含电机动力学 ---
    if isfield(baseQR, 'motorDynamicsIncluded') && baseQR.motorDynamicsIncluded
        % 包含电机 (6x66)
        Yi = regressorWithMotorDynamics(qi, dqi, ddqi);
    else
        % 仅刚体 (6x60)
        Yi = standard_regressor_UR10E(qi, dqi, ddqi);
    end
    
    % 降维映射: Yi * E -> Ybi
    Ybi = Yi * baseQR.permutationMatrix(:, 1:baseQR.numberOfBaseParameters);
    
    % 计算摩擦力部分
    Yfrctni = frictionRegressor(dqi);
    
    % 拼接并计算力矩
    tau_pred(i, :) = ([Ybi, Yfrctni] * pi_total)';
end

fprintf('计算完成！\n');

% =========================================================================
% 4. 绘图对比
% =========================================================================
figure('Name', 'Torque Prediction Check', 'Color', 'w');
for k = 1:6
    subplot(2, 3, k); hold on;
    plot(t, tau_meas(:, k), 'b', 'LineWidth', 1.2, 'DisplayName', 'Measured (Sim)');
    plot(t, tau_pred(:, k), 'r--', 'LineWidth', 1.2, 'DisplayName', 'Predicted');
    
    title(['Joint ', num2str(k)]);
    xlabel('Time (s)'); ylabel('Torque (Nm)');
    legend; grid on;
    xlim([t(1), t(end)]);
end

% =========================================================================
% 辅助函数区域 (直接写在脚本底部即可，或者作为独立函数)
% =========================================================================

% 1. 从 out 对象中提取原始变量
function raw = extract_signal_raw(out_obj, name)
    if isa(out_obj, 'Simulink.SimulationOutput')
        raw = out_obj.get(name);
    elseif isstruct(out_obj)
        raw = out_obj.(name);
    else
        error('未知的 out 类型');
    end
end

% 2. 从变量中提取数值矩阵 (解决你的报错)
function data = get_payload(val)
    if isa(val, 'timeseries')
        data = val.Data;
    elseif isstruct(val) && isfield(val, 'Data')
        data = val.Data;
    elseif isnumeric(val)
        % 如果已经是矩阵，直接返回，不要用 .Data
        data = val; 
    else
        error('无法识别的数据类型: %s', class(val));
    end
end