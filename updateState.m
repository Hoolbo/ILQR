function [f] = updateState(Xin,U,arg)
    x = Xin(1);
    y = Xin(2);
    phi = Xin(3);
    v = Xin(4);
    a = U(1);
    delta = U(2);
    X = [x;y;phi;v];
    U = [a;delta];
    beta = atan((arg.lr / (arg.lr + arg.lf)) * tan(delta));
    f = [x + v * cos(phi + beta) * arg.dt; 
        y + v * sin(phi + beta) * arg.dt; 
        phi + (v / arg.l) * tan(delta) * cos(beta) * arg.dt;
        v + a * arg.dt];
end