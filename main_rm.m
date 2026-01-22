clear ; close all; clc;

% Define path to a urdf file
path_to_urdf = 'URDF/RM65-6FB/urdf/RM65-6FB.urdf';


%% 在更换或更改机械臂URDF模型后取消注释运行，生成文件在/autogen
%%Generate functions for dynamics based on Lagrange method
%%Note that it might take some time
% generate_rb_dynamics(path_to_urdf);
% generate_friction_eq();


%%Generate regressors for inverse dynamics of the robot, friction and load
%%Note that it might take some time
% generate_rb_regressor(path_to_urdf);
% generate_load_regressor(path_to_urdf);


%%Run tests
% test_rb_inverse_dynamics(path_to_urdf)
% test_base_params()

%% 
% Perform QR decompostion in order to get base parameters of the robot
include_motor_dynamics = 1;
[pi_lgr_base, baseQR] = base_params_qr(include_motor_dynamics);


% Estimate drive gains
% drive_gains = estimate_drive_gains(baseQR, 'PC-OLS');
% Or use those found in the paper by De Luca
drive_gains = ones(6,1);

% Estimate dynamic parameters
fprintf('开始辨识...\n');
path_to_est_data = 'dataset_rm65fb\identification_data\N3_0122.csv';      idxs = [1, 4001];
% path_to_data = 'ur-20_02_12-40sec_12harm.csv';    idxs = [500, 4460];    
% path_to_data = 'ur-20_02_05-20sec_8harm.csv';     idxs = [320, 2310];
% path_to_data = 'ur-20_02_12-50sec_12harm.csv';    idxs = [355, 5090];
sol = estimate_dynamic_params(path_to_est_data, idxs, ...
                              drive_gains, baseQR, 'PC-OLS');

                         
% Validate estimated parameters
fprintf('开始验证...\n');
path_to_val_data = 'dataset_rm65fb\identification_data\N3_0122.csv';     idxs = [200, 2000];

rre = validate_dynamic_params(path_to_val_data, idxs, ...
                              drive_gains, baseQR, sol.pi_b, sol.pi_fr);



