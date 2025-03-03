% function [lx,lu,lxx,luu,lux] = getCostDerivatives(Xin,U)
% global arg
% lx = zeros(arg.N,arg.num_states);
% lxx = zeros(arg.N,arg.num_states,arg.num_states);
% lu = zeros(arg.N,arg.num_ctrl);
% luu = zeros(arg.N,arg.num_ctrl,arg.num_ctrl);
% lux = zeros(arg.N,arg.num_ctrl,arg.num_states); 
% 
% P1 = [1;0];
% P2 = [0;1];
% arg.totalBarrierCost = 0;
% for i=1:arg.N
%     %计算状态代价导数
%     [arg.x_r,arg.y_r,arg.theta_r]=findClosestPoint(Xin(i,:));
%     lx(i,:) = 2*arg.Q*[Xin(i,1)-arg.x_r; Xin(i,2)-arg.y_r; Xin(i,3)-arg.theta_r; Xin(i,4)-arg.desireSpeed;];
%     % lx(i,:) = 2*arg.Q*[Xin(i,1)-arg.x_r; Xin(i,2)-arg.y_r; 0; Xin(i,4)-arg.desireSpeed;];
%     lxx(i,:,:) = 2*arg.Q;
% 
% 
%     %计算控制代价导数
%     v = Xin(i,4);
%     c = U(i,:)*P2 - v * tan(arg.steer_angle_max/arg.l) ;
%     [b1, b1_dot, b1_ddot] = barrierFunction(10,3,c,P2);
% 
%     c = v * tan(arg.steer_angle_min/arg.l) - U(i,:)*P2;
%     [b2, b2_dot, b2_ddot] = barrierFunction(10,3,c,P2);
% 
%     arg.totalBarrierCost = arg.totalBarrierCost + b1 + b2;
%     lu(i,:) = 2*arg.R*[U(i,1);U(i,2)] + b1_dot + b2_dot;
%     luu(i,:,:) = 2*arg.R + b1_ddot + b2_ddot;
% end
% end
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
        %%%%%%%%%%% 计算状态代价导数
        [x_r, y_r, theta_r] = findClosestPoint(Xin(i,:));
        lx(i,:) = 2 * arg.Q * [Xin(i, 1) - x_r; Xin(i, 2) - y_r; Xin(i, 3) - theta_r; Xin(i, 4) - arg.desireSpeed];
        lxx(i,:,:) = 2 * arg.Q;
        % ---- 障碍物代价计算 ----
        [b_obs,b_dot_obs, b_ddot_obs] = obstacleCostDerivatives(Xin(i, 1), Xin(i, 2),obs_x,obs_y);
        obs_x(1) = obs_x(1) + arg.obs_dx;
        lx(i,:) = lx(i,:) + b_dot_obs;
        lxx(i,:,:) = lxx(i,:,:) + b_ddot_obs;
        
        
        %%%%%%%%% 计算控制代价导数
        %%%%转角约束
        v = Xin(i, 4);
        c = U(i,:) * P2 - v * tan(arg.steer_angle_max / arg.l);
        [b1, b1_dot, b1_ddot] = barrierFunction(arg.steer_max_q1, arg.steer_max_q2, c, P2);

        c = v * tan(arg.steer_angle_min / arg.l) - U(i,:) * P2;
        [b2, b2_dot, b2_ddot] = barrierFunction(arg.steer_min_q1, arg.steer_min_q2, c, P2);

        lu(i,:) = 2 * arg.R * [U(i, 1); U(i, 2)] + b1_dot + b2_dot;
        luu(i,:,:) = 2 * arg.R + b1_ddot + b2_ddot;
        
        %%计算总的约束代价
        arg.totalBarrierCost = arg.totalBarrierCost + b_obs + b1 + b2;
    end
end


