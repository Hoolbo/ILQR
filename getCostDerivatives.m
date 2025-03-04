function [lx, lu, lxx, luu, lux] = getCostDerivatives(Xin, U)
    global arg
    lx = zeros(arg.N, arg.num_states);
    lxx = zeros(arg.N, arg.num_states, arg.num_states);
    lu = zeros(arg.N, arg.num_ctrl);
    luu = zeros(arg.N, arg.num_ctrl, arg.num_ctrl);
    lux = zeros(arg.N, arg.num_ctrl, arg.num_states); 

    P1 = [1; 0];
    P2 = [0; 1];
    arg.totalBarrierCost = 0;
    obs_x = arg.obs_x;
    obs_y = arg.obs_y;
    for i = 1:arg.N
        %%%%%%%%%%% 计算代价对于状态的导数
        [x_r, y_r, theta_r] = findClosestPoint(Xin(i,:));
        lx(i,:) = 2 * arg.Q * [Xin(i, 1) - x_r; Xin(i, 2) - y_r; Xin(i, 3) - theta_r; Xin(i, 4) - arg.desireSpeed];
        lxx(i,:,:) = 2 * arg.Q;
        % ---- 障碍物代价计算 ----
        if arg.is_cal_obs_cost
            [~,db_obs, ddb_obs] = obstacle(Xin(i, 1), Xin(i, 2),obs_x,obs_y);
            obs_x(1) = obs_x(1) + arg.obs_dx;
        else
            db_obs = 0;
            ddb_obs = 0;
        end

        
        % ---- 道路边界代价计算 ----
        % 左侧约束
        if arg.is_cal_lane_cost
            dX = [Xin(i,1), Xin(i,2)] - [x_r, y_r];
            nor_r = [-sin(theta_r), cos(theta_r)];
            l = dX * nor_r';
            c_left = l - arg.trace_safe_width;
            dc_left = [-sin(theta_r), cos(theta_r),0,0];
            [~, db_lane_left, ddb_lane_left] = barrierFunction(arg.lane_q1, arg.lane_q2, c_left, dc_left);
            % 右侧约束
            c_right = -l - arg.trace_safe_width;  % 右侧约束，安全距离相同
            dc_right = -dc_left;  % 导数方向相反
            [~, db_lane_right, ddb_lane_right] = barrierFunction(arg.lane_q1, arg.lane_q2, c_right, dc_right);
            % 合并代价
            db_lane_total = db_lane_left + db_lane_right;
            ddb_lane_total = ddb_lane_left + ddb_lane_right;
        else
            db_lane_total = 0;
            ddb_lane_total = 0;
        end
        
        % 更新到代价导数
        lx(i,:) = lx(i,:) + db_obs + db_lane_total;
        lxx(i,:,:) = lxx(i,:,:)+ ddb_obs + ddb_lane_total;
        
        
        
        %%%%%%%%% 计算代价对于控制的导数
        %%%%转角约束
        if arg.is_cal_steer_cost
            % v = Xin(i, 4);
            % c = U(i,:) * P2 - v * tan(arg.steer_angle_max / arg.l);
            % [b1, db1, ddb1] = barrierFunction(arg.steer_max_q1, arg.steer_max_q2, c, P2);
            % 
            % c = v * tan(arg.steer_angle_min / arg.l) - U(i,:) * P2;
            % [b2, db2, ddb2] = barrierFunction(arg.steer_min_q1, arg.steer_min_q2, c, P2);
            c = U(i,:) * P2 - arg.steer_angle_max;
            [b1, db1, ddb1] = barrierFunction(arg.steer_max_q1, arg.steer_max_q2, c, P2);

            c = arg.steer_angle_min - U(i,:) * P2;
            [b2, db2, ddb2] = barrierFunction(arg.steer_min_q1, arg.steer_min_q2, c, -P2);

        else
            db1 = 0;
            db2 = 0;
            ddb1 = 0;
            ddb2 = 0;
        end
        
        lu(i,:) = 2 * arg.R * [U(i, 1); U(i, 2)] + db1 + db2;
        luu(i,:,:) = 2 * arg.R + ddb1 + ddb2;
        
%         %%计算总的约束代价
%         arg.totalBarrierCost = arg.totalBarrierCost + b_lane + b_obs + b1 + b2;
    end
end


