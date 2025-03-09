% function [] = Ploting(Xlog,Ulog, Xnew, Unew,i,arg,step_time,obs_traj)
%     persistent fig_handle car_handle traj_handle pred_handle obs_handles;
% 
%     % 初始化图形窗口
%     if isempty(fig_handle) || ~isvalid(fig_handle)
%         fig_handle = figure('Name', 'ILQR轨迹优化', 'NumberTitle', 'off','WindowState', 'maximized');
%         axis equal;
%         hold on;
%         zoom on;
%         xlabel('X Position (m)');
%         ylabel('Y Position (m)');
% 
%         % 绘制静态元素（仅一次）
%         color_lane = [0.678, 0.847, 0.902];
%         plot(arg.xcoord, arg.ycoord, '--', 'Color', [0.5,0.8,1], 'LineWidth', 1.2);
%         track_width = arg.trace_width;
%         theta_rad = arg.theta;
%         left_lane_x = arg.xcoord + track_width/2 * cos(theta_rad + pi/2);
%         left_lane_y = arg.ycoord + track_width/2 * sin(theta_rad + pi/2);
%         right_lane_x = arg.xcoord - track_width/2 * cos(theta_rad + pi/2);
%         right_lane_y = arg.ycoord - track_width/2 * sin(theta_rad + pi/2);
%         plot(left_lane_x, left_lane_y, '-', 'Color', color_lane, 'LineWidth', 1.5);
%         plot(right_lane_x, right_lane_y, '-', 'Color', color_lane, 'LineWidth', 1.5);
% 
%         % 初始化障碍物图形对象
%         obs_handles = gobjects(1, arg.obs_num); % 预分配句柄数组
%         for idx = 1:arg.obs_num
%             pos = [obs_traj(1,idx,1)-arg.obs_radius, obs_traj(1,idx,2)-arg.obs_radius,...
%                    2*arg.obs_radius, 2*arg.obs_radius];
%             obs_handles(idx) = rectangle('Position', pos, 'FaceColor', 'k', 'EdgeColor', 'none');
%         end
%     else
%         figure(fig_handle);
%     end
% 
%     % 动态更新障碍物位置（每次迭代更新）
%     if ~isempty(obs_handles)
%         for idx = 1:length(obs_handles)
%             if idx <= arg.obs_num && isvalid(obs_handles(idx))
%                 % 计算新位置
%                 new_pos = [obs_traj(1,idx,1)-arg.obs_radius, obs_traj(1,idx,2)-arg.obs_radius,...
%                    2*arg.obs_radius, 2*arg.obs_radius];
%                 % 更新图形属性
%                 set(obs_handles(idx), 'Position', new_pos);
%             end
%         end
%     end
% 
%     % --- 更新动态元素 ---
%     % 历史轨迹
%     if isempty(traj_handle) || ~isvalid(traj_handle)
%         traj_handle = line(nan, nan, 'Color', 'r', 'LineWidth', 1.5);
%     end
%     set(traj_handle, 'XData', Xlog(1:i,1), 'YData', Xlog(1:i,2));
% 
%     % 预测轨迹
%     if isempty(pred_handle) || ~isvalid(pred_handle)
%         pred_handle = scatter(nan, nan, 10, 'g', 'filled');
%     end
%     set(pred_handle, 'XData', Xnew(:,1), 'YData', Xnew(:,2));
% 
%     % 车辆矩形
%     x = Xnew(1,1);
%     y = Xnew(1,2);
%     theta_rad = Xnew(1,3);
%     dx = arg.l/2;
%     dy = arg.width/2;
%     rotation_matrix = [cos(theta_rad), sin(theta_rad); -sin(theta_rad), cos(theta_rad)];
%     corners = [-dx, -dy; dx, -dy; dx, dy; -dx, dy] * rotation_matrix;
%     corners = corners + [x, y];
% 
%     if isempty(car_handle) || ~isvalid(car_handle)
%         car_handle = fill(corners(:,1), corners(:,2), 'y', 'EdgeColor', 'k', 'LineWidth', 1);
%     else
%         set(car_handle, 'XData', corners(:,1), 'YData', corners(:,2));
%     end
% 
%     % --- 动态调整视图范围（新增代码）---
%     current_x = Xnew(1,1);
%     current_y = Xnew(1,2);
%     x_margin = 50;  % 控制横向视野范围
%     y_margin = 20;  % 控制纵向视野范围
%     speed = Xnew(1,4);
%     x_margin = max(100, speed * 3);  % 速度越大，边距越大
%     axis([current_x - x_margin, current_x + x_margin,...
%           current_y - y_margin, current_y + y_margin]);
%     % 定义平滑系数（0=完全跳变，1=保持原视图）
%     alpha = 0.2;
%     current_axis = axis();
%     target_axis = [current_x - x_margin, current_x + x_margin,...
%                    current_y - y_margin, current_y + y_margin];
%     smoothed_axis = alpha * target_axis + (1-alpha) * current_axis;
%     axis(smoothed_axis);
% 
%     % 更新标题
%     title(sprintf(['Iteration: %d  单步时间: %.f ms\n' ...
%         '           Speed: %.2f m/s  Accel: %.2f m/s²  Steer %.2f°  \n '], ...
%                   i,step_time * 1000,Xnew(1,4), Unew(1,1), rad2deg(Unew(1,2))));
%     drawnow;
% end
function [] = Ploting(Xlog, Ulog, Xnew, Unew, i, arg, step_time, obs_traj)
    persistent fig_handle sub1 sub2 sub3 car_handle traj_handle pred_handle obs_handles accel_plot steer_plot

    %% 图形初始化
    if isempty(fig_handle) || ~isvalid(fig_handle)
        % 创建图形窗口
        fig_handle = figure('Name', 'ILQR轨迹优化', 'NumberTitle', 'off', 'WindowState', 'maximized');
        
        % 主轨迹图（占大部分区域）
        sub1 = subplot('Position', [0.1 0.3 0.8 0.6]); % 主图位置
        axis(sub1, 'equal'); hold(sub1, 'on');
        xlabel('X Position (m)'); ylabel('Y Position (m)');
        
        % 绘制道路元素
        plot(arg.xcoord, arg.ycoord, '--', 'Color', [0.5, 0.8, 1], 'LineWidth', 1.2, 'Parent', sub1);
        track_width = arg.trace_width;
        left_lane_x = arg.xcoord + track_width / 2 * cos(arg.theta + pi / 2);
        left_lane_y = arg.ycoord + track_width / 2 * sin(arg.theta + pi / 2);
        right_lane_x = arg.xcoord - track_width / 2 * cos(arg.theta + pi / 2);
        right_lane_y = arg.ycoord - track_width / 2 * sin(arg.theta + pi / 2);
        plot(sub1, left_lane_x, left_lane_y, '-', 'Color', [0.678, 0.847, 0.902], 'LineWidth', 1.5);
        plot(sub1, right_lane_x, right_lane_y, '-', 'Color', [0.678, 0.847, 0.902], 'LineWidth', 1.5);
        
        % 初始化障碍物
        obs_handles = gobjects(1, arg.obs_num);
        for idx = 1:arg.obs_num
            pos = [obs_traj(1, idx, 1) - arg.obs_radius, obs_traj(1, idx, 2) - arg.obs_radius, ...
                   2 * arg.obs_radius, 2 * arg.obs_radius];
            obs_handles(idx) = rectangle('Position', pos, 'FaceColor', 'k', 'EdgeColor', 'none', 'Parent', sub1);
        end
        
        % 加速度子图（左下角）
        sub2 = subplot('Position', [0.1 0.1 0.35 0.15]); % 左下角
        accel_plot = plot(sub2, nan, nan, 'b-', 'LineWidth', 1.5);
        xlabel('迭代次数'); ylabel('加速度 (m/s²)');
        grid on; title('加速度历史');
        xlim([1 10]); % 初始范围
        
        % 转向角子图（右下角）
        sub3 = subplot('Position', [0.55 0.1 0.35 0.15]); % 右下角
        steer_plot = plot(sub3, nan, nan, 'r-', 'LineWidth', 1.5);
        xlabel('迭代次数'); ylabel('转向角 (°)');
        grid on; title('转向角历史');
        xlim([1 10]); % 初始范围
    else
        figure(fig_handle); % 切换到现有图形窗口
    end

    %% 更新主轨迹图
    % 更新障碍物位置
    for idx = 1:arg.obs_num
        if isvalid(obs_handles(idx)) % 检查对象是否有效
            new_pos = [obs_traj(1, idx, 1) - arg.obs_radius, obs_traj(1, idx, 2) - arg.obs_radius, ...
                       2 * arg.obs_radius, 2 * arg.obs_radius];
            set(obs_handles(idx), 'Position', new_pos);
        end
    end
    % 更新预测轨迹
    if isempty(pred_handle) || ~isvalid(pred_handle)
        pred_handle = scatter(sub1,Xnew(:,1), Xnew(:,2), 10, 'g', 'filled');
    end
        set(pred_handle, 'XData', Xnew(:,1), 'YData', Xnew(:,2));

    % 更新历史轨迹
    if isempty(traj_handle) || ~isvalid(traj_handle)
        traj_handle = plot(sub1, Xlog(1:i, 1), Xlog(1:i, 2), 'r-', 'LineWidth', 1.5);
    else
        set(traj_handle, 'XData', Xlog(1:i, 1), 'YData', Xlog(1:i, 2));
    end
    
    % 更新车辆模型
    x = Xnew(1, 1); y = Xnew(1, 2); theta = Xnew(1, 3);
    car_corners = [-arg.l / 2, -arg.width / 2; arg.l / 2, -arg.width / 2; ...
                   arg.l / 2, arg.width / 2; -arg.l / 2, arg.width / 2] * ...
                  [cos(theta), sin(theta); -sin(theta), cos(theta)] + [x, y];
    if isempty(car_handle) || ~isvalid(car_handle)
        car_handle = fill(sub1, car_corners(:, 1), car_corners(:, 2), 'y', 'EdgeColor', 'k');
    else
        set(car_handle, 'XData', car_corners(:, 1), 'YData', car_corners(:, 2));
    end
    
    %% 更新控制量曲线
    % 加速度曲线
    set(accel_plot, 'XData', 1:i, 'YData', Ulog(1:i, 1));
    sub2.XLim = [1 max(10, i + 2)]; % 动态调整范围
    sub2.YLim = [min(Ulog(1:i, 1)) - 0.5, max(Ulog(1:i, 1)) + 0.5];
    
    % 转向角曲线（转换为度数）
    set(steer_plot, 'XData', 1:i, 'YData', rad2deg(Ulog(1:i, 2)));
    sub3.XLim = [1 max(10, i + 2)]; % 动态调整范围
    sub3.YLim = [min(rad2deg(Ulog(1:i, 2))) - 5, max(rad2deg(Ulog(1:i, 2))) + 5];
    
    %% 主图视野调整
    x_margin = max(50, Xnew(1, 4) * 3);
    y_margin = 30;
    axis(sub1, [x - x_margin, x + x_margin, y - y_margin, y + y_margin]);
    
    %% 更新标题
    title(sub1, sprintf('Iteration: %d  单步时间: %.1f ms  速度: %.2f m/s', ...
          i, step_time * 1000, Xnew(1, 4)));
    
    drawnow limitrate; % 限制刷新频率
end