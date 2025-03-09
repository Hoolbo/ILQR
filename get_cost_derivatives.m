function [lx, lu, lxx, luu, lux] = get_cost_derivatives(X_traj, U, local_plan, obs_traj,arg)

    lx = zeros(arg.N+1, arg.num_states);
    lxx = zeros(arg.N+1, arg.num_states, arg.num_states);
    lu = zeros(arg.N, arg.num_ctrl);
    luu = zeros(arg.N, arg.num_ctrl, arg.num_ctrl);
    lux = zeros(arg.N, arg.num_ctrl, arg.num_states); 
    
    P1 = [1; 0];
    P2 = [0; 1];
    arg.totalBarrierCost = 0;

    for i = 1:arg.N+1
        %%%%%%%%%%% 计算代价对于状态的导数
        [x_r, y_r, theta_r] = findClosestPoint(X_traj(i,:),local_plan);
        l_dx = 2 * arg.Q * [X_traj(i, 1) - x_r; X_traj(i, 2) - y_r; X_traj(i, 3) - theta_r; X_traj(i, 4) - arg.desireSpeed];
        l_ddx = 2 * arg.Q;
        
        obs_x = obs_traj(i,:,1);
        obs_y = obs_traj(i,:,2);
        % ---- 障碍物代价计算 ----
        if arg.is_cal_obs_cost
            [~,db_obs, ddb_obs] = obstacle(X_traj(i, 1), X_traj(i, 2),obs_x,obs_y,arg);
        else
            db_obs = 0;
            ddb_obs = 0;
        end

        % ---- 道路边界代价计算 ----
        % 左侧约束
        if arg.is_cal_lane_cost
            dX = [X_traj(i,1), X_traj(i,2)] - [x_r, y_r];
            nor_r = [-sin(theta_r), cos(theta_r)];
            l = dX * nor_r';
            c_left = l - arg.trace_safe_width_left;
            dc_left = [-sin(theta_r); cos(theta_r);0;0];
            [~, db_lane_left, ddb_lane_left] = barrierFunction(arg.lane_q1, arg.lane_q2, c_left, dc_left);
            % 右侧约束
            c_right = -l - arg.trace_safe_width_right;  % 右侧约束，安全距离相同
            dc_right = -dc_left;  % 导数方向相反
            [~, db_lane_right, ddb_lane_right] = barrierFunction(arg.lane_q1, arg.lane_q2, c_right, dc_right);
            % 合并代价
            db_lane_total = db_lane_left + db_lane_right;
            ddb_lane_total = ddb_lane_left + ddb_lane_right;
        else
            db_lane_total = 0;
            ddb_lane_total = 0;
        end
   
        % 合并
        lx(i,:) = l_dx + db_obs + db_lane_total;
        lxx(i,:,:) = l_ddx+ ddb_obs + ddb_lane_total;
    end
    
    for i = 1:arg.N
            %%%%%%%%% 计算代价对于控制的导数
            if arg.is_cal_steer_cost
                c = U(i,:) * P2 - arg.steer_angle_max;
                [~, db1, ddb1] = barrierFunction(arg.steer_max_q1, arg.steer_max_q2, c, P2);

                c = arg.steer_angle_min - U(i,:) * P2;
                [~, db2, ddb2] = barrierFunction(arg.steer_min_q1, arg.steer_min_q2, c, -P2);
            else
                db1 = 0;
                db2 = 0;
                ddb1 = 0;
                ddb2 = 0;
            end

            lu(i,:) = 2 * arg.R * [U(i, 1); U(i, 2)] + db1 + db2;
            luu(i,:,:) = 2 * arg.R + ddb1 + ddb2;
    end
    
end


