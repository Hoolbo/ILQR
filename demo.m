function demo
    clc
    close all
    global arg
    %载入参数
    arguments();
    %%截取地图
    map_start_index = 1;
    mat_end_index = length(arg.xcoord);
    arg.xcoord = arg.xcoord(map_start_index:mat_end_index);
    arg.ycoord = arg.ycoord(map_start_index:mat_end_index);
    arg.slength = arg.slength(map_start_index:mat_end_index);
    arg.Curv    = arg.Curv(map_start_index:mat_end_index) ;
    arg.theta   = arg.theta(map_start_index:mat_end_index) ;
    %状态变量 x y phi v
    X = [270; 10 ; 0 ; arg.startSpeed;];

    Xlog = zeros(arg.num_states,arg.tf/arg.dt);

    %控制变量 a 加速度 delta 前轮转角
    U = zeros(arg.num_ctrl);

    for i=1:arg.tf/arg.dt

        fprintf('===================== 仿真第%d步 ==================\n ',i);
        tic
        [Xnew,Unew] = ilqr(X);
        toc
        X = updateState(Xnew(1,:),Unew(1,:));
        Xlog(1:2,i) = X(1:2);
        fprintf('速度V=%f\n前轮转角=%f\n',X(4),rad2deg(Unew(1,2)));

%         if (mod(i,2) == 0)

        Ploting(Xlog,Xnew,i);
%         end
%         fprintf('---------- 画图花费时间 ------------\n ',i);


    end
end


