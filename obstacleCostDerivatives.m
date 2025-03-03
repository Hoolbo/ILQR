function [b,b_dot, b_ddot] = obstacleCostDerivatives(x, y)
    global arg
    % 计算与障碍物的距离
    dx = x - arg.obs_x;  % Ego车与障碍物的x坐标差
    dy = y - arg.obs_y;  % Ego车与障碍物的y坐标差
    dist = sqrt(dx^2 + dy^2);       % 计算距离

    % 障碍物代价计算：若距离小于安全阈值，产生代价
    safe_distance = arg.obs_radius + arg.ego_rad;  % 安全距离
    % c小于0满足约束  c大于0违反约束
    c = safe_distance - dist;   
    % 一阶导数
    c_dot = [-1,-1,0,0];
        % 返回代价的导数
    [b1,b1_dot,b1_ddot] = barrierFunction(arg.obs_q1,arg.obs_q2,c,c_dot);
    
        %%%%%%%%%%%%% 计算与障碍物2的距离
    dx = x - arg.obs_x2;  % Ego车与障碍物的x坐标差
    dy = y - arg.obs_y2;  % Ego车与障碍物的y坐标差
    dist = sqrt(dx^2 + dy^2);       % 计算距离
    % 障碍物约束计算：若距离小于安全阈值，产生代价
    safe_distance = arg.obs_radius + arg.ego_rad;  % 安全距离
    % c小于0满足约束  c大于0违反约束
    c = safe_distance - dist;   
    % 一阶导数
    c_dot = [-1,-1,0,0];
    % 返回屏障代价以及导数
    [b2,b2_dot,b2_ddot] = barrierFunction(arg.obs_q1,arg.obs_q2,c,c_dot);
    b2 = 0;
    b2_dot = 0;
    b2_ddot = 0;
    
    
    b = b1 + b2;
    b_dot = b1_dot + b2_dot;
    b_ddot = b1_ddot + b2_ddot;
end
