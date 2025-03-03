function[Jnew] = getTotalCost(X,U)
%暂定
Jnew = 0;
global arg
    %%计算和参考线的状态差代价和控制代价
        P2 = [0; 1];
        obs_x = arg.obs_x;
        obs_y = arg.obs_y;
        for i=1:arg.N
            [x_r,y_r,theta_r]= findClosestPoint(X(i+1,:));
            ref_state = [x_r; y_r; 0 ; arg.desireSpeed;];
            state_diff = X(i+1,:)' - ref_state;

            cost_state = state_diff' * arg.Q * state_diff;
            cost_ctrl = U(i,:) * arg.R * U(i,:)';
            
            [b_obs,b_dot_obs, b_ddot_obs] = obstacleCostDerivatives(X(i, 1), X(i, 2),obs_x,obs_y);
            obs_x(1) = obs_x(1) + arg.obs_dx;
            v = X(i, 4);
            c = U(i,:) * P2 - v * tan(arg.steer_angle_max / arg.l);
            [b1, b1_dot, b1_ddot] = barrierFunction(arg.steer_max_q1, arg.steer_max_q2, c, P2);

            c = v * tan(arg.steer_angle_min / arg.l) - U(i,:) * P2;
            [b2, b2_dot, b2_ddot] = barrierFunction(arg.steer_min_q1, arg.steer_min_q2, c, P2);

            cost_constraint = b_obs + b1 + b2;

            Jnew = Jnew + cost_state + cost_ctrl + cost_constraint ; 
        end

end