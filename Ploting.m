% function [] = Ploting(Xlog,Xnew,i)
%     global arg
%     clf;
%     hold on;
%     axis equal;
%     
%     % ----------------------------
%     % 1. 绘制地图背景
%     % ----------------------------
% %     scatter(arg.xcoord, arg.ycoord, 50, [0.678, 0.847, 0.902], 'filled'); % 地图散点
%     % 使用预定义颜色（颜色名称或缩写）
%     color_lane = [0.678, 0.847, 0.902];
%     plot(arg.xcoord, arg.ycoord, '--', 'Color', color_lane, 'LineWidth', 1.2);
%     % ----------------------------
%     % 2. 计算并绘制车道线
%     % ----------------------------
%     % 假设赛道宽度为4米（根据实际情况调整）
%     track_width = 4; 
%     
%     % 计算左右车道线坐标
%     theta_rad = arg.theta;
%     left_lane_x = arg.xcoord + track_width/2 * cos(theta_rad + pi/2);
%     left_lane_y = arg.ycoord + track_width/2 * sin(theta_rad + pi/2);
%     right_lane_x = arg.xcoord - track_width/2 * cos(theta_rad + pi/2);
%     right_lane_y = arg.ycoord - track_width/2 * sin(theta_rad + pi/2);
%     
%     % 绘制车道线（白色实线）
%     plot(left_lane_x, left_lane_y, '-', 'Color',color_lane,'LineWidth', 1.5); 
%     plot(right_lane_x, right_lane_y, '-','Color', color_lane,'LineWidth', 1.5);
%     
%     % ----------------------------
%     % 3. 绘制其他动态元素
%     % ----------------------------
%     % 行驶轨迹（红色散点）
%     scatter(Xlog(1,1:i), Xlog(2,1:i), 5, 'red', 'filled');
%     
%     % ILQR预测轨迹（绿色散点）
%     scatter(Xnew(:,1), Xnew(:,2), 5, 'green', 'filled');
%     
% %     % 当前车辆位置（黄色标记）
% %     scatter(Xnew(1,1), Xnew(1,2), 100, 'yellow', 'o', 'LineWidth', 1.5);
%      % 车辆中心位置和航向角（直接从Xnew获取）
%     x = Xnew(1,1);          % X坐标（第1列）
%     y = Xnew(1,2);          % Y坐标（第2列）
%     theta_rad = Xnew(1,3);  % 航向角（第4列，单位：弧度）
%     
%     % 计算旋转后的矩形顶点
%     dx = arg.l/2;
%     dy = arg.width/2;
%     rotation_matrix = [cos(theta_rad), sin(theta_rad); 
%                        -sin(theta_rad),  cos(theta_rad)];
%     corners = [-dx, -dy; dx, -dy; dx, dy; -dx, dy];
%     rotated_corners = corners * rotation_matrix;
%     rotated_corners(:,1) = rotated_corners(:,1) + x;
%     rotated_corners(:,2) = rotated_corners(:,2) + y;
%     
%     % 绘制车辆（黄色填充，黑色边框）
%     fill(rotated_corners(:,1), rotated_corners(:,2), 'y', 'EdgeColor', 'k', 'LineWidth', 1);
%     
%     % 障碍物（黑色矩形）
%     rectangle('Position', [300, 0, 2, 1], 'EdgeColor', 'k', 'FaceColor', 'k', 'LineWidth', 0.5);
%     
%     % ----------------------------
%     % 4. 图形优化
%     % ----------------------------
%     title(['Iteration: ', num2str(i)]);
%     xlabel('X Position (m)');
%     ylabel('Y Position (m)');
%     drawnow;
% end


function [] = Ploting(Xlog, Xnew, i)
    global arg
    persistent fig_handle car_handle traj_handle pred_handle;

    % 初始化图形窗口
    if isempty(fig_handle) || ~isvalid(fig_handle)
        fig_handle = figure('Name', 'ILQR轨迹优化', 'NumberTitle', 'off');
        axis equal;
        hold on;
        zoom on;
        xlabel('X Position (m)');
        ylabel('Y Position (m)');
        
        % 绘制静态元素（仅一次）
        color_lane = [0.678, 0.847, 0.902];
        plot(arg.xcoord, arg.ycoord, '--', 'Color', [0.5,0.8,1], 'LineWidth', 1.2);
        track_width = 4;
        theta_rad = arg.theta;
        left_lane_x = arg.xcoord + track_width/2 * cos(theta_rad + pi/2);
        left_lane_y = arg.ycoord + track_width/2 * sin(theta_rad + pi/2);
        right_lane_x = arg.xcoord - track_width/2 * cos(theta_rad + pi/2);
        right_lane_y = arg.ycoord - track_width/2 * sin(theta_rad + pi/2);
        plot(left_lane_x, left_lane_y, '-', 'Color', color_lane, 'LineWidth', 1.5);
        plot(right_lane_x, right_lane_y, '-', 'Color', color_lane, 'LineWidth', 1.5);
        %绘制障碍物
        for i=1:length(arg.obs_x)
            rectangle('Position', [arg.obs_x(i) - arg.obs_radius(i), arg.obs_y(i) - arg.obs_radius(i), arg.obs_radius(i) * 2, arg.obs_radius(i)*2], 'FaceColor', 'k');
        end

    else
        figure(fig_handle);
    end

    % --- 更新动态元素（修复逻辑）---
    % 历史轨迹（使用line对象）
    if isempty(traj_handle) || ~isvalid(traj_handle)
        traj_handle = line(nan, nan, 'Color', 'r', 'LineWidth', 1.5);
    end
%     valid_indices = max(1, i-100):i; % 显示最近100步轨迹
    valid_indices = 1:i; % 显示最近100步轨迹
    set(traj_handle, 'XData', Xlog(1,valid_indices), 'YData', Xlog(2,valid_indices));

    % 预测轨迹（使用scatter）
    if isempty(pred_handle) || ~isvalid(pred_handle)
        pred_handle = scatter(nan, nan, 10, 'g', 'filled');
    end
    set(pred_handle, 'XData', Xnew(:,1), 'YData', Xnew(:,2));

    % 车辆矩形（同前）
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

    title(['Iteration: ', num2str(i)]);
    drawnow;
end