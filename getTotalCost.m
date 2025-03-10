function[J_total] = getTotalCost(X,U,local_plan,obs_traj,arg)
        %%计算和参考线的状态差代价和控制代价
        P2 = [0; 1];
        J_state_total = 0;
        J_ctrl_total = 0;
        J_constraint_total = 0;
        
        %%计算状态代价
        for i=1:arg.N+1
             [x_r,y_r,theta_r]= findClosestPoint(X(i,:),local_plan);
             ref_state = [x_r; y_r; 0 ; arg.desireSpeed];
             state_diff = X(i,:)' - ref_state;
             cost_state = state_diff' * arg.Q * state_diff;
             obs_x = obs_traj(i,:,1);
             obs_y = obs_traj(i,:,2);
             if arg.is_cal_obs_cost
                %%计算障碍物代价
               [cost_obs,~, ~] = obstacle(X(i, 1), X(i, 2),obs_x,obs_y,arg);
             else
                cost_obs = 0;
             end
             
             if arg.is_cal_lane_cost
                %%左侧车道偏离代价
                dX = [X(i,1),X(i,2)] - [x_r,y_r];
                nor_r = [-sin(theta_r),cos(theta_r)];
                l = dX * nor_r';
                c_left = l - arg.trace_safe_width_left;
                cost_lane_left = arg.lane_q1*exp(arg.lane_q2*c_left);
                %%右侧车道偏离代价
                c_right = -l - arg.trace_safe_width_right;  % 右侧约束，安全距离相同
                cost_lane_right = arg.lane_q1*exp(arg.lane_q2*c_right);
                cost_lane = cost_lane_left + cost_lane_right;
             else
                cost_lane = 0;
             end
             
             J_state_total = J_state_total + cost_state;
             J_constraint_total = J_constraint_total + cost_obs + cost_lane;
        end
        
        %%计算控制代价
        for i=1:arg.N
            cost_ctrl = U(i,:) * arg.R * U(i,:)';
            
            if arg.is_cal_steer_cost
                c = U(i,:) * P2 - arg.steer_angle_max;
                b1 = arg.steer_max_q1*exp(arg.steer_max_q2*c);

                c = arg.steer_angle_min - U(i,:) * P2;
                b2 = arg.steer_min_q1*exp(arg.steer_min_q2*c);
                
                cost_steer_limit = b1 + b2;
            else
                cost_steer_limit = 0;
            end
            
            J_ctrl_total = J_ctrl_total + cost_ctrl;
            J_constraint_total = J_constraint_total + cost_steer_limit;
        end
        J_total = J_state_total + J_ctrl_total + J_constraint_total;
end