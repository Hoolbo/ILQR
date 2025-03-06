function[Xnominal,Unominal] = getNominalTrajectory(X_cur,local_plan,arg)

Xnominal = zeros(arg.N+1,arg.num_states);
%规划起点赋值
Xnominal(1,:) = X_cur(:);
Unominal = ones(arg.N,arg.num_ctrl) * 0.1;

if isnan(arg.preU)
    %没有上一帧的控制序列 用纯跟踪获取控制序列并roll out出轨迹
    for i=1:arg.N
        Unominal(i,2) = purePursuit(X_cur,local_plan,arg);
        Xout = updateState(X_cur,Unominal(i,:),arg);
        Xnominal(i+1,:) = Xout(:);
        X_cur = Xout;
    end
else
    %有上一帧的控制序列，复用上一帧的控制，从上一帧的第二个信号取到最后一个，
    Unominal(1:end-1,:) = arg.preU(2:end,:);
    %当前帧的最后一个控制信号赋值为上一帧的最后一个信号
    Unominal(end,:) = Unominal(end,:);

    %用获得的控制信号rollout出轨迹
    for i=1:arg.N
        Xout = updateState(X_cur,Unominal(i,:));
        Xnominal(i+1,:) = Xout(:);
        X_cur = Xout;
    end
end
end

% function [Xnominal, Unominal] = getNominalTrajectory(Xin)
%     global arg
%     if ~isnan(arg.preU)
%         % 复用上一帧控制序列的后半部分
%         Unominal = [arg.preU(2:end,:); arg.preU(end,:)];
%     else
%         % 使用速度控制+ Pure Pursuit
%         Unominal(:,1) = (arg.desireSpeed - Xin(4)) * 0.3; % 加速度控制
%         for i=1:arg.N
%             Unominal(i,2) = purePursuit(Xin);
%             Xin = updateState(Xin, Unominal(i,:));
%         end
%     end
%     % Rollout轨迹
%     for i=1:arg.N
%         Xout = updateState(Xin,Unominal(i,:));
%         Xnominal(i+1,:) = Xout(:);
%         Xin = Xout;
%     end
% 
% end
% 
% function[Xnominal] = rollout(X0,Unominal)
%      global arg
%      Xnominal(1,:) = X0;
%      X = X0;
%      for i=1:arg.N
%         X = updateState(X,Unominal(i,:));
%         Xnominal(i+1,:) = X;
%      end
% end