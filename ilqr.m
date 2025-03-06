function[X,U,arg] = ilqr(X0,arg)

    %%获取局部路径
    local_plan = getLocalPlan(X0,arg);

    %%获取粗解（粗控制和粗轨迹）
    [X,U] = getNominalTrajectory(X0,local_plan,arg);

    %%求粗解的代价
    Jold = getTotalCost(X,U,local_plan,arg);

    %%初始正则化系数lamb
    lamb = arg.lamb_init;
    
    
    for i=1:arg.max_iter
        %%反向传播获得反馈系数
        [k,K] = backward(X,U,lamb,local_plan,arg);
        %%正向传播roll out出新轨迹
        [Xnew,Unew] = forward(X,U,k,K,local_plan,arg);
        %%求新轨迹和控制的代价
        Jnew = getTotalCost(Xnew,Unew,local_plan,arg);
    %     fprintf('Iter %d | λ=%.2f | Jold-Jnew=%.6f | Jold=%.2f\n',i,lamb,Jold-Jnew,Jold);

        %%收敛判断
        if Jnew <Jold 
            X = Xnew;
            U = Unew;
            % lamb = lamb / arg.lamb_factor;
            % 修改后：根据成本下降比例调整λ
            if (Jold - Jnew)/Jold > 0.1  % 成本显著下降
                lamb = max(lamb / 2, 1e-6); % 激进降低λ
            else
                lamb = min(lamb * 5, arg.lamb_max); % 保守增加λ
            end
            if (Jold - Jnew)/Jold < arg.rel_tol
                arg.error_count = 0;
                fprintf('迭代第%d次，求解成功\n',i);
                arg.preX = X;
                arg.preU = U;
                break;
            end
            Jold = Jnew;
        else           
            lamb = lamb * arg.lamb_factor;
            if lamb > arg.lamb_max
                arg.error_count = arg.error_count + 1;
                fprintf('迭代第%d次，求解失败\n',i);
                arg.preX = arg.preX_nan;
                arg.preU = arg.preX_nan;
                break;
            end
        end
    end
end
