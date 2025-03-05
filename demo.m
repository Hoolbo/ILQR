function demo

    %% 初始化
    %载入参数
    global arg
    arguments();
    %状态变量 x X坐标 y Y坐标 phi 航向角 v 速度
    X0 = [ 200 ; 1.24 ; 0 ; arg.startSpeed;];
    %控制变量 a 加速度 delta 前轮转角（弧度）
    % U = [ a ; delta ];
    %记录行驶过的轨迹
    Xlog = zeros(arg.num_states,arg.tf/arg.dt);

    %% 仿真
    for i=1:arg.tf/arg.dt
        fprintf('===================== 仿真第%d步 ==================\n ',i);
        arg.obs_x(1) = arg.obs_x(1) + arg.obs_dx;
        tic
        [X_trajectory,U_sequence] = ilqr(X0);
        toc
        if arg.error_count >5
            error("连续求解失败次数大于10，停止仿真");
        end
        X0 = updateState(X0,U_sequence(1,:));
        Xlog(1:2,i) = X0(1:2);

        %画图
        Ploting(Xlog,X_trajectory,U_sequence,i);
    end
end


