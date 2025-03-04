function [Xout] = updateState(Xin,U)
    global arg
    x = Xin(1);
    y = Xin(2);
    phi = Xin(3);
    v = Xin(4);
    a = U(1);
    delta = U(2);
    beta = atan((arg.lr / (arg.lr + arg.lf)) * tan(delta));
    xout = x + v * cos(phi + beta) * arg.dt;
    yout = y + v * sin(phi + beta) * arg.dt;
    phiout = phi + (v / arg.l) * (beta) * arg.dt;
    vout = v + a * arg.dt;
    Xout = [xout; yout; phiout; vout];
end