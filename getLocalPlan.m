function [] = getLocalPlan(Xin)
global arg
    temp = (Xin(1)-arg.xcoord(:)).^2 + (Xin(2)-arg.ycoord(:)).^2;
    mintemp = min(temp);
    index = find(mintemp == temp);
    arg.localPlan = zeros(arg.N,3);
    arg.localPlanIndex = index;
    for i=1:100
    arg.localPlan(i,1) = arg.xcoord(index+i);
    arg.localPlan(i,2) = arg.ycoord(index+i);
    arg.localPlan(i,3) = arg.theta(index+i);
    end
end