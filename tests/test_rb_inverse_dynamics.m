function test_rb_inverse_dynamics(path_to_urdf)
    fprintf('\n==========================================================\n');
    fprintf('[TEST] 正在运行刚体逆动力学验证 (Gravity Check)...\n');
    fprintf('==========================================================\n');

    % 1. 加载参数
    % 这里的 parse_urdf 负责提取质量、质心、惯量等参数
    ur10 = parse_urdf(path_to_urdf);
    
    % 2. 加载 MATLAB 机器人模型
    % [修正] 使用传入的路径，而不是硬编码字符串，防止文件版本不一致
    try
        rbt = importrobot(path_to_urdf); 
    catch
        error('importrobot 无法加载路径: %s，请检查文件是否存在或路径是否正确。', path_to_urdf);
    end
    
    rbt.DataFormat = 'column';
    
    % [关键调试点 1] 重力向量
    rbt.Gravity = [9.81,0,0]; 
    fprintf('MATLAB Robotics Toolbox Gravity set to: [%.2f %.2f %.2f]\n', rbt.Gravity);

    no_iter = 3; % 先只跑5次，方便看打印
    
    for i = 1:no_iter
        fprintf('\n--- Iteration %d ---\n', i);
        
        % 生成随机关节角
        q = -2*pi + 4*pi*rand(6,1);
        
        % [关键调试点 2] 仅测试静态重力 (速度加速度设为0)
        % 这样可以排除 Coriolis 和 Inertia 矩阵的干扰
        q_d = zeros(6,1);
        q_2d = zeros(6,1);
        
        % --- A. 计算 MATLAB 官方结果 ---
        tau_matlab = inverseDynamics(rbt, q, q_d, q_2d);
        
        % --- B. 计算你的回归矩阵结果 ---
        % 注意：这里的 standard_regressor_UR10E 是你刚刚生成的那个函数
        Ylgr = standard_regressor_UR10E(q, q_d, q_2d);
        tau_reg = Ylgr * reshape(ur10.pi, [60,1]);
        
        % --- C. 计算你的动力学矩阵结果 ---
        tau_manip = M_mtrx_fcn(q, ur10.pi(:))*q_2d + ...
                    C_mtrx_fcn(q, q_d, ur10.pi(:))*q_d + ...
                    G_vctr_fcn(q, ur10.pi(:));
        
        % --- D. 打印对比 ---
        fprintf('Joint |  Matlab ToolBox  |  Your Regressor  |  Your Eq(M/C/G)  |   Diff (Reg) \n');
        fprintf('-----------------------------------------------------------------------------\n');
        for j = 1:6
            diff_val = tau_matlab(j) - tau_reg(j);
            fprintf('  J%d  | %14.6f   | %14.6f   | %14.6f   | %14.6e\n', ...
                j, tau_matlab(j), tau_reg(j), tau_manip(j), diff_val);
        end
        
        % 错误检查
        err_reg = norm(tau_matlab - tau_reg);
        err_manip = norm(tau_matlab - tau_manip);
        
        if err_reg > 1e-5 || err_manip > 1e-5
            fprintf('\n[FAIL] 误差过大! (Norm Err: %.6f)\n', err_reg);
            fprintf('建议检查:\n');
            fprintf('1. 重力方向是否相反？(观察数值是否互为相反数)\n');
            fprintf('2. parse_urdf 是否正确读取了质量(mass)和质心(CoM)?\n');
            error('Test Failed at iteration %d', i);
        end
    end
    
    fprintf("\n[PASS] Rigid Body Inverse Dynamics Test - OK! (Gravity Term matches)\n");
end