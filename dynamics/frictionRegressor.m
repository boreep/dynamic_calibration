function frctn = frictionRegressor(qd_fltrd)
% ----------------------------------------------------------------------
% The function computes friction regressor for each joint of the robot.
% Fv*qd + Fc*sign(qd) + F0, and the second one is continous,
% ---------------------------------------------------------------------
alpha = 100; % 平滑系数，值越大越接近 sign 函数，建议取 10-100 之间

noJoints = size(qd_fltrd,1);
frctn = zeros(noJoints, noJoints*3);
for i = 1:noJoints
    % 使用 tanh 替换 sign
    frctn(i,3*i-2:3*i) = [qd_fltrd(i), tanh(alpha * qd_fltrd(i)), 1];
end

