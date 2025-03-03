% function [df_dx,df_du] = ilqr_getABMatrix(Xin,Uin)
% 
% global arg
% syms sym_x sym_y sym_phi sym_v sym_a sym_delta
% beta = atan((arg.lr / (arg.lr + arg.lf)) * tan(sym_delta));
% xout = sym_x + sym_v * cos(sym_phi * beta) * arg.dt;
% yout = sym_y + sym_v * sin(sym_phi * beta) * arg.dt;
% phiout = sym_phi + (sym_v / arg.lf) * sin(beta) * arg.dt;
% vout = sym_v + sym_a * arg.dt;
% f = [xout; yout; phiout; vout];
% X = [sym_x,sym_y,sym_phi,sym_v];
% U = [sym_a,sym_delta];
% df_dx_sym = jacobian(f, X);
% df_du_sym = jacobian(f, U);
% 
% df_dx = zeros(arg.N,arg.num_states,arg.num_states);
% df_du = zeros(arg.N,arg.num_states,arg.num_ctrl);
% for i=1:arg.N
%     vars = [sym_x sym_y sym_phi sym_v sym_a sym_delta];
%     values = [Xin(i,1) Xin(i,2) Xin(i,3) Xin(i,4) Uin(i,1) Uin(i,2)];
%     df_dx(i,:,:) = double(subs(df_dx_sym, vars, values));
%     df_du(i,:,:) = double(subs(df_du_sym, vars, values));
% end
% 
% end
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
            x(3) + (x(4) / arg.lr) * sin(beta) * arg.dt;
            x(4) + u(1) * arg.dt;
        ];
    end

    % 计算雅可比矩阵（有限差分）
    delta = 1e-5; % 设置扰动值
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
