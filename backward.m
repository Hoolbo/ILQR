function[k,K] = backward(X,U,lamb,local_plan,obs_traj,arg)
%%求系统动力学方程对状态x，控制u的雅可比矩阵
[df_dx,df_du] = get_dynamics_jacobians(X(1:end-1,:),U,arg);
%%求代价函数l对x,u的各阶导数
[lx,lu,lxx,luu,lux] = get_cost_derivatives(X,U,local_plan,obs_traj,arg);
%%初始化V_x和V_xx为最终步的代价函数l对x的雅可比矩阵和海森矩阵
V_x = lx(end,:)';
V_xx = squeeze(lxx(end,:,:))';
%%初始化k,K数组
k = zeros(arg.N, arg.num_ctrl);
K = zeros(arg.N, arg.num_ctrl, arg.num_states);
%%主循环
for i=arg.N:-1:1
    df_dx_N = squeeze(df_dx(i,:,:));
    df_du_N = squeeze(df_du(i,:,:));
    df_dx_T = df_dx_N';
    df_du_T = df_du_N';
    
    Qx = squeeze(lx(i,:))' + df_dx_T * V_x;
    Qu = squeeze(lu(i,:))' + df_du_T * V_x;
    Qxx = squeeze(lxx(i,:,:)) + df_dx_T * V_xx * df_dx_N;
    Qux = squeeze(lux(i,:,:)) + df_du_T * V_xx * df_dx_N;
    Quu = squeeze(luu(i,:,:)) + df_du_T * V_xx * df_du_N;


    [U_Q_uu, s_Q_uu, V_Q_uu] = svd(Quu);
    s_Q_uu = max(diag(s_Q_uu),1e-8) + lamb;
    Quu_inv = V_Q_uu * diag(1.0./s_Q_uu) * U_Q_uu';

    k(i,:) = -Quu_inv * Qu;
    K(i,:,:) = -Quu_inv *Qux;
    V_x = Qx - squeeze(K(i,:,:))' * Quu * k(i,:)';
    V_xx = Qxx - squeeze(K(i,:,:))' * Quu * squeeze(K(i,:,:));

end
end
