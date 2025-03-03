function[Xnominal,U] = getNominalTrajectory(Xin)
global arg

Xnominal = zeros(arg.N+1,arg.num_states);
%%规划起点赋值
Xnominal(1,:) = Xin(:);
U = zeros(arg.N,arg.num_ctrl) ;

% if isnan(arg.preU)
    for i=1:arg.N
        U(i,2) = purePursuit(Xin);
        Xout = updateState(Xin,U(i,:));
        Xnominal(i+1,:) = Xout(:);
        Xin = Xout;
    end
% else
%     Xnominal = arg.preX;
%     U = arg.preU;
% end

end