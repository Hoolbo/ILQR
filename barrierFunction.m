function [b,b_dot,b_ddot] = barrierFunction(q1,q2,c,c_dot)
    
    	b = q1*exp(q2*c);
		b_dot = q1*q2*exp(q2*c)*c_dot;
		b_ddot = q1*(q2^2)*exp(q2*c)*(c_dot * c_dot');

end