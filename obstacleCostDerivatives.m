function [b,b_dot, b_ddot] = obstacleCostDerivatives(x, y)
    global arg
    b = 0;
    b_dot = 0;
    b_ddot = 0;
    for i = 1:length(arg.obs_x)
        % 计算与障碍物的距离
        dx = x - arg.obs_x(i);  % Ego车与障碍物的x坐标差
        dy = y - arg.obs_y(i);  % Ego车与障碍物的y坐标差
        dist = sqrt(dx^2 + dy^2);       % 计算距离

        % 障碍物代价计算：若距离小于安全阈值，产生代价
        safe_distance = arg.obs_radius(i) + arg.ego_rad;  % 安全距离
        % c小于0满足约束  c大于0违反约束
        c = safe_distance - dist;   
        % 一阶导数
        c_dot = [-1,-1,0,0];
        % 返回代价的导数
        [b1,b1_dot,b1_ddot] = barrierFunction(arg.obs_q1,arg.obs_q2,c,c_dot);
        
        b = b + b1 ;
        b_dot = b_dot + b1_dot ;
        b_ddot = b_ddot + b1_ddot ;
    end

    

end
