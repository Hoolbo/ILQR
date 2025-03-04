function[Jnew] = getTotalCost(X,U)
%暂定
Jnew = 0;
global arg
    %%计算和参考线的状态差代价和控制代价
        P2 = [0; 1];
        obs_x = arg.obs_x;
        obs_y = arg.obs_y;
        cost_constraint = 0;
        for i=1:arg.N
            [x_r,y_r,theta_r]= findClosestPoint(X(i+1,:));
            ref_state = [x_r; y_r; 0 ; arg.desireSpeed];
            state_diff = X(i+1,:)' - ref_state;

            cost_state = state_diff' * arg.Q * state_diff;
            cost_ctrl = U(i,:) * arg.R * U(i,:)';
            if arg.is_cal_obs_cost
                %%计算障碍物代价
               [b_obs,~, ~] = obstacle(X(i, 1), X(i, 2),obs_x,obs_y);
            else
                b_obs = 0;
            end
            obs_x(1) = obs_x(1) + arg.obs_dx;
            
            
            if arg.is_cal_lane_cost
                %%左侧车道偏离代价
                dX = [X(i,1),X(i,2)] - [x_r,y_r];
                nor_r = [-sin(theta_r),cos(theta_r)];
                l = dX * nor_r';
                c_left = l - arg.trace_safe_width;
                b_lane_left = arg.lane_q1*exp(arg.lane_q2*c_left);
                %%右侧车道偏离代价
                c_right = -l - arg.trace_safe_width;  % 右侧约束，安全距离相同
                b_lane_right = arg.lane_q1*exp(arg.lane_q2*c_right);
                b_lane = b_lane_left + b_lane_right;
            else
                b_lane = 0;
            end

            if arg.is_cal_steer_cost
                c = U(i,:) * P2 - arg.steer_angle_max;
                b1 = arg.steer_max_q1*exp(arg.steer_max_q2*c);

                c = arg.steer_angle_min - U(i,:) * P2;
                b2 = arg.steer_min_q1*exp(arg.steer_min_q2*c);
                
                b_steer_limit = b1 + b2;
            else
                b_steer_limit = 0;
            end


            cost_constraint = b_obs + b_lane + b_steer_limit;

            Jnew = Jnew + cost_state + cost_ctrl + cost_constraint; 
        end

end