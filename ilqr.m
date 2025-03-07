function[X,U,success] = ilqr(X0,obs_traj,arg)

    %%获取局部路径
    local_plan = getLocalPlan(X0,arg);

    %%获取粗解（粗控制和粗轨迹）并求代价
    [X,U] = getNominalTrajectory(X0,local_plan,arg);
    Jold = getTotalCost(X,U,local_plan,obs_traj,arg);

    %%初始正则化系数lamb
    lamb = arg.lamb_init;
    
    %%主循环
    for i=1:arg.max_iter
        fprintf('迭代第%d次\n',i);
        %%反向传播获得反馈系数
        [k,K] = backward(X,U,lamb,local_plan,obs_traj,arg);
        %%正向传播roll out出新轨迹
        [Xnew,Unew,Jnew] = forward(X,U,k,K,local_plan,obs_traj,arg);

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
                fprintf('迭代第%d次，求解成功\n',i);
                success = true;
                break;
            end
            Jold = Jnew;
        else           
            lamb = lamb * arg.lamb_factor;
            if lamb > arg.lamb_max
                fprintf('迭代第%d次，超过lamb最大值求解失败\n',i);
                success = false;
                break;
            end
        end

    end

    if i == arg.max_iter 
                fprintf('迭代第%d次，超过最大迭代次数求解失败\n',i);
                success = false;
    end
end
