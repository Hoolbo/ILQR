function[k,K] = backward(X,U,lamb,local_plan,arg)

[lx,lu,lxx,luu,lux] = getCostDerivatives(X(2:end,:),U,local_plan,arg);
[df_dx,df_du] = ilqr_getABMatrix(X(2:end,:),U,arg);


V_x = lx(end,:)';
V_xx = squeeze(lxx(end,:,:))';
k = zeros(arg.N, arg.num_ctrl);
K = zeros(arg.N, arg.num_ctrl, arg.num_states);

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
    s_Q_uu = diag(s_Q_uu);
    s_Q_uu = max(s_Q_uu, 0);
    s_Q_uu = max(s_Q_uu,1e-8) + lamb + 1e-6;

    Quu_inv = U_Q_uu' * diag(1.0./s_Q_uu) * V_Q_uu';

    k(i,:) = -Quu_inv * Qu;
    K(i,:,:) = -Quu_inv *Qux;
    V_x = Qx - squeeze(K(i,:,:))' * Quu * k(i,:)';
    V_xx = Qxx - squeeze(K(i,:,:))' * Quu * squeeze(K(i,:,:));

end
end
