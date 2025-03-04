function [b,db,ddb] = barrierFunction(q1,q2,c,c_dot)
    
    	b = q1*exp(q2*c);
		db = q1*q2*exp(q2*c)*c_dot;
		ddb = q1*(q2^2)*exp(q2*c)*(c_dot * c_dot');

end