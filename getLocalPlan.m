function [local_plan] = getLocalPlan(X0,arg)
    temp = (X0(1)-arg.xcoord(:)).^2 + (X0(2)-arg.ycoord(:)).^2;
    mintemp = min(temp);
    index = find(mintemp == temp);
    local_plan = zeros(arg.N,5);
    localPlanIndex = index + 1;
    for i=1:100
    local_plan(i,1) = arg.xcoord(localPlanIndex + i);
    local_plan(i,2) = arg.ycoord(localPlanIndex + i);
    local_plan(i,3) = arg.theta(localPlanIndex + i);
    local_plan(i,3) = arg.slength(localPlanIndex+ i);
    local_plan(i,3) = arg.Curv(localPlanIndex+ i);
    end
end