% function [] = Ploting(Xlog, Xnew, Unew,i)
%     global arg
%     persistent fig_handle car_handle traj_handle pred_handle;
% 
%     % 初始化图形窗口
%     if isempty(fig_handle) || ~isvalid(fig_handle)
%         fig_handle = figure('Name', 'ILQR轨迹优化', 'NumberTitle', 'off');
%         axis equal;
%         hold on;
%         zoom on;
%         xlabel('X Position (m)');
%         ylabel('Y Position (m)');
% 
%         % 绘制静态元素（仅一次）
%         color_lane = [0.678, 0.847, 0.902];
%         plot(arg.xcoord, arg.ycoord, '--', 'Color', [0.5,0.8,1], 'LineWidth', 1.2);
%         track_width = 4;
%         theta_rad = arg.theta;
%         left_lane_x = arg.xcoord + track_width/2 * cos(theta_rad + pi/2);
%         left_lane_y = arg.ycoord + track_width/2 * sin(theta_rad + pi/2);
%         right_lane_x = arg.xcoord - track_width/2 * cos(theta_rad + pi/2);
%         right_lane_y = arg.ycoord - track_width/2 * sin(theta_rad + pi/2);
%         plot(left_lane_x, left_lane_y, '-', 'Color', color_lane, 'LineWidth', 1.5);
%         plot(right_lane_x, right_lane_y, '-', 'Color', color_lane, 'LineWidth', 1.5);
%         %绘制障碍物
%         for i=1:length(arg.obs_x)
%             rectangle('Position', [arg.obs_x(i) - arg.obs_radius(i), arg.obs_y(i) - arg.obs_radius(i), arg.obs_radius(i) * 2, arg.obs_radius(i)*2], 'FaceColor', 'k');
%         end
% 
%     else
%         figure(fig_handle);
%     end
% 
%     % --- 更新动态元素（修复逻辑）---
%     % 历史轨迹（使用line对象）
%     if isempty(traj_handle) || ~isvalid(traj_handle)
%         traj_handle = line(nan, nan, 'Color', 'r', 'LineWidth', 1.5);
%     end
% %     valid_indices = max(1, i-100):i; % 显示最近100步轨迹
%     valid_indices = 1:i; % 显示最近100步轨迹
%     set(traj_handle, 'XData', Xlog(1,valid_indices), 'YData', Xlog(2,valid_indices));
% 
%     % 预测轨迹（使用scatter）
%     if isempty(pred_handle) || ~isvalid(pred_handle)
%         pred_handle = scatter(nan, nan, 10, 'g', 'filled');
%     end
%     set(pred_handle, 'XData', Xnew(:,1), 'YData', Xnew(:,2));
% 
%     % 车辆矩形（同前）
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
%     title(sprintf('Iteration: %d\n   Speed: %.2f m/s           Accel: %.2f m/s^2           Steer %.1f °', i,Xnew(1,4),Unew(1,1),rad2deg(Unew(1,2))));
%     drawnow;
% end
function [] = Ploting(Xlog, Xnew, Unew,i)
    global arg
    persistent fig_handle car_handle traj_handle pred_handle obs_handles;

    % 初始化图形窗口
    if isempty(fig_handle) || ~isvalid(fig_handle)
        fig_handle = figure('Name', 'ILQR轨迹优化', 'NumberTitle', 'off');
        axis equal;
        hold on;
        zoom on;
        xlabel('X Position (m)');
        ylabel('Y Position (m)');
        
        % --- 新增：计算参考路径的范围并设置坐标轴 ---
        xmin = min(arg.xcoord) - 50;  % 左边距50米
        xmax = max(arg.xcoord) + 50;  % 右边距50米
        ymin = min(arg.ycoord) - 20;  % 下边距20米
        ymax = max(arg.ycoord) + 20;  % 上边距20米
        axis([xmin xmax ymin ymax]);   % 固定显示范围
        
        % 绘制静态元素（仅一次）
        color_lane = [0.678, 0.847, 0.902];
        plot(arg.xcoord, arg.ycoord, '--', 'Color', [0.5,0.8,1], 'LineWidth', 1.2);
        track_width = arg.trace_width;
        theta_rad = arg.theta;
        left_lane_x = arg.xcoord + track_width/2 * cos(theta_rad + pi/2);
        left_lane_y = arg.ycoord + track_width/2 * sin(theta_rad + pi/2);
        right_lane_x = arg.xcoord - track_width/2 * cos(theta_rad + pi/2);
        right_lane_y = arg.ycoord - track_width/2 * sin(theta_rad + pi/2);
        plot(left_lane_x, left_lane_y, '-', 'Color', color_lane, 'LineWidth', 1.5);
        plot(right_lane_x, right_lane_y, '-', 'Color', color_lane, 'LineWidth', 1.5);
        
        % 初始化障碍物图形对象
        obs_handles = gobjects(1, length(arg.obs_x)); % 预分配句柄数组
        for idx = 1:length(arg.obs_x)
            pos = [arg.obs_x(idx)-arg.obs_radius(idx), arg.obs_y(idx)-arg.obs_radius(idx),...
                   2*arg.obs_radius(idx), 2*arg.obs_radius(idx)];
            obs_handles(idx) = rectangle('Position', pos, 'FaceColor', 'k', 'EdgeColor', 'none');
        end
    else
        figure(fig_handle);
    end

    % 动态更新障碍物位置（每次迭代更新）
    if ~isempty(obs_handles)
        for idx = 1:length(obs_handles)
            if idx <= length(arg.obs_x) && isvalid(obs_handles(idx))
                % 计算新位置
                new_pos = [arg.obs_x(idx)-arg.obs_radius(idx),...
                          arg.obs_y(idx)-arg.obs_radius(idx),...
                          2*arg.obs_radius(idx), 2*arg.obs_radius(idx)];
                % 更新图形属性
                set(obs_handles(idx), 'Position', new_pos);
            end
        end
    end

    % --- 更新动态元素 ---
    % 历史轨迹
    if isempty(traj_handle) || ~isvalid(traj_handle)
        traj_handle = line(nan, nan, 'Color', 'r', 'LineWidth', 1.5);
    end
    set(traj_handle, 'XData', Xlog(1,1:i), 'YData', Xlog(2,1:i));

    % 预测轨迹
    if isempty(pred_handle) || ~isvalid(pred_handle)
        pred_handle = scatter(nan, nan, 10, 'g', 'filled');
    end
    set(pred_handle, 'XData', Xnew(:,1), 'YData', Xnew(:,2));

    % 车辆矩形
    x = Xnew(1,1);
    y = Xnew(1,2);
    theta_rad = Xnew(1,3);
    dx = arg.l/2;
    dy = arg.width/2;
    rotation_matrix = [cos(theta_rad), sin(theta_rad); -sin(theta_rad), cos(theta_rad)];
    corners = [-dx, -dy; dx, -dy; dx, dy; -dx, dy] * rotation_matrix;
    corners = corners + [x, y];
    
    if isempty(car_handle) || ~isvalid(car_handle)
        car_handle = fill(corners(:,1), corners(:,2), 'y', 'EdgeColor', 'k', 'LineWidth', 1);
    else
        set(car_handle, 'XData', corners(:,1), 'YData', corners(:,2));
    end
    
    % 更新标题
    title(sprintf('Iteration: %d\nSpeed: %.2f m/s  Accel: %.2f m/s²  Steer %.1f°',...
                  i, Xnew(1,4), Unew(1,1), rad2deg(Unew(1,2))));
    drawnow;
end