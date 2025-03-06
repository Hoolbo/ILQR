function [u] = purePursuit(X_cur,local_plan,arg)

% 1. 计算最近点索引（避免多个索引）
[~, indexNow] = min((X_cur(1) - local_plan(:,1)).^2 + (X_cur(2) - local_plan(:,2)).^2);

% 2. 计算前瞻距离（限制范围）
Ld = max(arg.Ld_min, min(arg.Ld_max, arg.Kv * X_cur(4) + arg.Ld0));

% 3. 查找目标点（避免越界）
numPoints = length(local_plan(:,1));
targetDist = local_plan(indexNow,1) + Ld;
indexTarget = find(local_plan(:,4) >= targetDist, 1, 'first');

% 处理目标点超出路径的情况
if isempty(indexTarget) || indexTarget > numPoints
    indexTarget = numPoints;
end

% 4. 计算转向角（角度归一化）
dx = local_plan(indexTarget,1) - X_cur(1);
dy = local_plan(indexTarget,2) - X_cur(2);
alpha = wrapToPi(atan2(dy, dx) - X_cur(3));

% 5. 计算前轮转角
u = atan2(2 * arg.l * sin(alpha), Ld);

end