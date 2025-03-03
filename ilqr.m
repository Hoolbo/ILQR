function[X,U] = ilqr(Xin)
global arg

% --------------getLocalPlan-------------------------
getLocalPlan(Xin);
% --------------getLocalPlan-------------------------

[Xnominal,U] = getNominalTrajectory(Xin);
arg.Xnominal = Xnominal;
X = Xnominal;

Jold = getTotalCost(Xnominal,U);
lamb = arg.lamb_init;

for i=1:arg.max_iter
    
    [k,K] = backward(X,U,lamb);
    
    [Xnew,Unew] = forward(X,U,k,K);
    
    Jnew = getTotalCost(Xnew,Unew);
%     fprintf('Iter %d | λ=%.2f | Jold-Jnew=%.6f | Jold=%.2f\n',i,lamb,Jold-Jnew,Jold);
    if Jnew <Jold 
        X = Xnew;
        U = Unew;
        lamb = lamb / arg.lamb_factor;
        arg.preX = X;
        arg.preU = U;
        if (Jold - Jnew)/Jold < arg.rel_tol
            fprintf('迭代第%d次，求解成功\n',i);
            break;
        end
        Jold = Jnew;
    else           
        lamb = lamb * arg.lamb_factor;
        if(Jnew - Jold)<1e-5
            fprintf('迭代第%d次，求解成功\n',i);
            break
        end
        if lamb > arg.lamb_max
            fprintf('迭代第%d次，求解失败\n',i);
            break;
        end
    end
    
end


end
