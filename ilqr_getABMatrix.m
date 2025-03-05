function [df_dx, df_du] = ilqr_getABMatrix(Xin, Uin)
    global arg
    % 预分配雅可比矩阵
    df_dx = zeros(arg.N, arg.num_states, arg.num_states);
    df_du = zeros(arg.N, arg.num_states, arg.num_ctrl);
    
    % 定义系统动力学
    function f = dynamics(x, u)
        beta = atan((arg.lr / (arg.lr + arg.lf)) * tan(u(2)));
        f = [
            x(1) + x(4) * cos(x(3) + beta) * arg.dt;
            x(2) + x(4) * sin(x(3) + beta) * arg.dt;
            x(3) + (x(4) / arg.l) * tan(u(2)) * cos(beta) * arg.dt;
            x(4) + u(1) * arg.dt;
        ];
    end

    % 计算雅可比矩阵（有限差分）
    delta = 1e-6; % 设置扰动值
    for i = 1:arg.N
        x0 = Xin(i, :)';
        u0 = Uin(i, :)';

        % 计算 df_dx
        for j = 1:arg.num_states
            dx = zeros(arg.num_states, 1);
            dx(j) = delta;
            df_dx(i, :, j) = (dynamics(x0 + dx, u0) - dynamics(x0, u0)) / delta;
        end

        % 计算 df_du
        for j = 1:arg.num_ctrl
            du = zeros(arg.num_ctrl, 1);
            du(j) = delta;
            df_du(i, :, j) = (dynamics(x0, u0 + du) - dynamics(x0, u0)) / delta;
        end
    end
    
end

