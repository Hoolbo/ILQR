 function demo

    %% 初始化
    %载入参数
    arg = arguments();
    %状态变量 x X坐标 y Y坐标 phi 航向角 v 速度
    start_index = 1;
    X0 = [ arg.xcoord( start_index) ; arg.ycoord( start_index) ; arg.theta( start_index) ; arg.startSpeed;];

    %控制变量 a 加速度 delta 前轮转角（弧度）
    % U = [ a ; delta ];
    %记录行驶过的轨迹
    Xlog = zeros(arg.tf/arg.dt,arg.num_states);
    Ulog = zeros(arg.tf/arg.dt,arg.num_ctrl);
    error_count = 0;
    %% 初始化障碍物轨迹
    obs_traj = arg.obs_traj;
    %% 仿真
    for i=1:arg.tf/arg.dt
        fprintf('===================== 仿真第%d步 ==================\n ',i);
        tic
        %% ILQR迭代获取最优控制序列和rollout出相应轨迹
        [X_trajectory,U_sequence,success] = ilqr(X0,obs_traj,arg);
        step_time = toc;

        if success == true
            error_count = 0;
            arg.preU = U_sequence;
        else
            error_count = error_count + 1;
            arg.preU = arg.preU_nan;
            if error_count >5
            error("连续求解失败次数大于10，停止仿真");
            end
        end
        %%记录历史轨迹数据
        Xlog(i,:) = X0;
        Ulog(i,:) = U_sequence(1,:);

        %画图
        Ploting(Xlog,Ulog,X_trajectory,i,arg,step_time,obs_traj);
        %%更新车辆状态和障碍物状态
        X0 = X_trajectory(2,:);
        obs_traj = update_obs_state(arg.obs_num,obs_traj,arg.N);
    end
end


