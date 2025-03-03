function demo
    clc
    close all
    global arg
    %载入参数
    arguments();
    %状态变量 x X坐标 y Y坐标 phi 航向角 v 速度
    X = [ 270 ; 1.24 ; 0 ; arg.startSpeed;];
    %控制变量 a 加速度 delta 前轮转角（弧度）
    % U = [ a ; delta ];
    %% 记录行驶过的轨迹
    Xlog = zeros(arg.num_states,arg.tf/arg.dt);

    for i=1:arg.tf/arg.dt
        fprintf('===================== 仿真第%d步 ==================\n ',i);
        tic
        [Xnew,Unew] = ilqr(X);
        toc
        X = updateState(Xnew(1,:),Unew(1,:));
        Xlog(1:2,i) = X(1:2);
        fprintf('速度V=%f\n前轮转角=%f\n',X(4),rad2deg(Unew(1,2)));
        %画图
        Ploting(Xlog,Xnew,i);
    end
end


