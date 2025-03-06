function [Xnew, Unew] = forward(X, U, k, K,local_plan,arg)
    
    persistent last_alpha  % 持久变量保存上一次成功的alpha
    if isempty(last_alpha)
        last_alpha = 1;  % 初始化为1
    end
    
    JoldForward = getTotalCost(X, U,local_plan,arg);
    alpha = min(last_alpha * 1.1, 1);  % 动态调整初始alpha
    Xnew = zeros(arg.N+1, arg.num_states);
    Xnew(1, :) = X(1, :);
    Unew = zeros(arg.N, arg.num_ctrl);
    found = false;
    
    max_iterations = 20;  % 减少最大循环次数
    min_alpha = 1e-4;     % 设置最小alpha阈值
    
    for j = 1:max_iterations
        % 前向传播计算新状态和控制序列
        for i = 1:arg.N
            Unew(i, :) = U(i, :)' + alpha * k(i, :)' + squeeze(K(i, :, :)) * (Xnew(i, :) - X(i, :))';
            Xnew(i+1, :) = updateState(Xnew(i, :), Unew(i, :),arg);
        end
        
        JnewForward = getTotalCost(Xnew, Unew,local_plan,arg);
        
        if JnewForward <= JoldForward
            last_alpha = alpha;  % 记录成功alpha
            found = true;
            break;
        else
            alpha = alpha * 0.5;  % 衰减因子保持0.5
            if alpha < min_alpha  % 提前终止条件
                break;
            end
        end
    end
    
    if ~found
        last_alpha = 1;  % 重置alpha为1，避免后续迭代受影响
%         error('Line search failed: No suitable alpha found after %d iterations.', max_iterations);
    end
    

end
