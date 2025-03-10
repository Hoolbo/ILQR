function [df_dx, df_du] = get_dynamics_jacobians(Xin, Uin, arg)
    % 预分配雅可比矩阵
    df_dx = zeros(arg.N, arg.num_states, arg.num_states);
    df_du = zeros(arg.N, arg.num_states, arg.num_ctrl);
    % 计算雅可比矩阵（有限差分）
    for i = 1:arg.N
        x0 = Xin(i, :)';
        u0 = Uin(i, :)';
        % 计算 df_dx
            phi = x0(3);
            v = x0(4);
            delta = u0(2);
            beta = atan((arg.lr / (arg.lr + arg.lf)) * tan(delta));
            
            df_dx(i, :, :) = [1, 0, -v*sin(phi+beta)*arg.dt,cos(phi+beta)*arg.dt;
                              0, 1,  v*cos(phi+beta)*arg.dt,sin(phi+beta)*arg.dt;
                              0, 0, 1, tan(delta)*cos(beta) * arg.dt / arg.l    ; 
                              0, 0, 0, 1                                       ];
        % 计算 df_du
            phi = x0(3);
            v = x0(4);
            delta = u0(2);
            beta = atan((arg.lr / (arg.lr + arg.lf)) * tan(delta));
            k = arg.lr / arg.l; 
            dbeta_ddelta = k * sec(delta) * sec(delta) / (1 + k*k*tan(delta)*tan(delta));
            df_du(i,:,:) = [  0  ,                         -v*sin(phi+beta)*dbeta_ddelta * arg.dt                             ;
                              0  ,                          v*cos(phi+beta)*dbeta_ddelta * arg.dt                             ;
                              0  ,  (v/arg.l) * arg.dt * (sec(delta)*sec(delta)*cos(beta) - tan(delta)*sin(beta)*dbeta_ddelta);
                         arg.dt  ,                                          0                                                ];
    end
    
end

