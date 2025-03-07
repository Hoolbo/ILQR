function[x,y,theta,index] = findClosestPoint(X,local_plan)
    temp = (X(1)-local_plan(:,1)).^2 + (X(2)-local_plan(:,2)).^2;
    mintemp = min(temp);
    index = find(mintemp == temp);
    x = local_plan(index,1);
    y = local_plan(index,2);
    theta = local_plan(index,3);
end