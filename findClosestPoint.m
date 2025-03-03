function[x,y,theta,index] = findClosestPoint(X)
    global arg
    temp = (X(1)-arg.localPlan(:,1)).^2 + (X(2)-arg.localPlan(:,2)).^2;
    mintemp = min(temp);
    index = find(mintemp == temp);
    x = arg.localPlan(index,1);
    y = arg.localPlan(index,2);
    theta = arg.localPlan(index,3);
end