%% 将 Simulink 仿真保存的 MAT 文件转换为符合 parseURData 的 CSV
clear; clc;

% -------------------------------------------------------------------------
% 1. 设置输入文件
% -------------------------------------------------------------------------
% 在此处修改你要处理的 mat 文件名
mat_filename = '10s_verify_out.mat'; 

% 自动生成输出文件名
[pathstr, name, ext] = fileparts(mat_filename);
csv_filename = fullfile(pathstr, [name, '.csv']);

% -------------------------------------------------------------------------
% 2. 加载 MAT 文件
% -------------------------------------------------------------------------
fprintf('正在加载 %s ...\n', mat_filename);
if ~isfile(mat_filename)
    error('找不到文件: %s，请确认文件名或路径是否正确。', mat_filename);
end

loaded_data = load(mat_filename);

% 检查 MAT 文件中是否存在变量 'out'
% 注意：load 返回的是 struct，所以这里可以用 isfield
if isfield(loaded_data, 'out')
    out = loaded_data.out;
    fprintf('成功加载变量 "out" (类型: %s)。\n', class(out));
else
    vars = fieldnames(loaded_data);
    error('在 MAT 文件中未找到变量 "out"。\n文件包含的变量有: %s', strjoin(vars, ', '));
end

% -------------------------------------------------------------------------
% 3. 提取数据 (专门适配 SimulationOutput)
% -------------------------------------------------------------------------
% 使用辅助函数安全提取，避免使用 isfield
t   = get_time_vector(out);
q   = extract_signal_data(out, 'q');    % [N x 6]
dq  = extract_signal_data(out, 'dq');   % [N x 6]
tau = extract_signal_data(out, 'tor');  % [N x 6]

data_len = length(t);
fprintf('数据提取完成，长度: %d\n', data_len);

% -------------------------------------------------------------------------
% 4. 构建矩阵并填空
% -------------------------------------------------------------------------
csv_data = zeros(data_len, 31); 

csv_data(:, 1)    = t;       % 时间
csv_data(:, 2:7)  = q;       % 关节位置
csv_data(:, 8:13) = dq;      % 关节速度

% 把力矩填入第 14-19 列 (对应电机电流位置)
csv_data(:, 14:19) = tau;    

% 把力矩也填到 tau_des 位置 (26-31列)
csv_data(:, 26:31) = tau;    

% -------------------------------------------------------------------------
% 5. 保存为 CSV
% -------------------------------------------------------------------------
writematrix(csv_data, csv_filename); 

fprintf('转换完成！\n目标文件: %s\n', csv_filename);


% =========================================================================
% 辅助函数定义
% =========================================================================

function val = extract_signal_data(out_obj, sig_name)
    % 从 out 对象中提取信号数据的通用函数
    raw_val = [];
    
    % 1. 判断容器类型并提取原始变量
    if isa(out_obj, 'Simulink.SimulationOutput')
        % 情况 A: SimulationOutput 对象 -> 使用 get 方法
        try
            raw_val = out_obj.get(sig_name);
        catch
            error('在 SimulationOutput 对象中找不到信号: %s', sig_name);
        end
    elseif isstruct(out_obj)
        % 情况 B: 普通 Struct -> 使用 isfield 和 点号访问
        if isfield(out_obj, sig_name)
            raw_val = out_obj.(sig_name);
        else
            error('在 Struct 中找不到字段: %s', sig_name);
        end
    else
        error('未知的 out 变量类型: %s', class(out_obj));
    end
    
    % 2. 从原始变量中提取数值 (.Data)
    % 处理 Timeseries 或 StructWithTime 等常见 Simulink 格式
    if isa(raw_val, 'timeseries')
        val = raw_val.Data;
    elseif isstruct(raw_val) && isfield(raw_val, 'Data')
        val = raw_val.Data;
    elseif isnumeric(raw_val)
        val = raw_val; % 已经是矩阵
    else
        warning('信号 %s 的格式未知 (%s)，尝试直接使用。', sig_name, class(raw_val));
        val = raw_val;
    end
end

function t = get_time_vector(out_obj)
    % 专门提取时间向量，尝试 'tout' 和 't'
    t = [];
    
    if isa(out_obj, 'Simulink.SimulationOutput')
        % 尝试获取 tout
        try
            t = out_obj.get('tout');
        catch
            try
                t = out_obj.get('t');
            catch
                % 如果都没有，尝试从 q 里拿时间
                try
                    q_ts = out_obj.get('q');
                    if isa(q_ts, 'timeseries')
                        t = q_ts.Time;
                        fprintf('警告: 未找到 tout，已使用 q.Time 作为时间向量。\n');
                    end
                catch
                end
            end
        end
    elseif isstruct(out_obj)
        if isfield(out_obj, 'tout')
            t = out_obj.tout;
        elseif isfield(out_obj, 't')
            t = out_obj.t;
        end
    end
    
    if isempty(t)
        error('无法在 out 中找到时间变量 (t 或 tout)');
    end
end