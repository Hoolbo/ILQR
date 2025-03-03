% function [u] = purePursuit(Xin)
% global arg
% 
% temp = (Xin(1)-arg.xcoord(:)).^2 + (Xin(2)-arg.ycoord(:)).^2;
% mintemp = min(temp);
% indexNow = find(mintemp == temp);
% 
% indexTarget = indexNow(1);
% L = 0;
% Ld = arg.Kv * Xin(4) + arg.Ld0;
% while L < Ld
%     L = L + (arg.slength(indexTarget+1)-arg.slength(indexTarget));
%     indexTarget = indexTarget+1;
% end
% 
% if indexTarget >= 10000
%     indexTarget = 10000;
% end
%     
% alpha = atan2(arg.ycoord(indexTarget)-Xin(2),arg.xcoord(indexTarget)-Xin(1)) - Xin(3);
% 
% u = atan2(2*arg.l*sin(alpha),Ld);
% 
% end

function [u] = purePursuit(Xin)
global arg

% 1. 计算最近点索引（避免多个索引）
[~, indexNow] = min((Xin(1) - arg.xcoord(:)).^2 + (Xin(2) - arg.ycoord(:)).^2);

% 2. 计算前瞻距离（限制范围）
Ld = max(arg.Ld_min, min(arg.Ld_max, arg.Kv * Xin(4) + arg.Ld0));

% 3. 查找目标点（避免越界）
numPoints = length(arg.xcoord);
targetDist = arg.slength(indexNow) + Ld;
indexTarget = find(arg.slength >= targetDist, 1, 'first');

% 处理目标点超出路径的情况
if isempty(indexTarget) || indexTarget > numPoints
    indexTarget = numPoints;
end

% 4. 计算转向角（角度归一化）
dx = arg.xcoord(indexTarget) - Xin(1);
dy = arg.ycoord(indexTarget) - Xin(2);
alpha = wrapToPi(atan2(dy, dx) - Xin(3));

% 5. 计算前轮转角
u = atan2(2 * arg.l * sin(alpha), Ld);

end