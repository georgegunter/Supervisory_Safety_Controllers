function [accel] = TTC(a_perf,s,v,dv,p)
%This may not be implemented properly...


tau=p(1);

A = 0;
if(dv>=(1/(tau*s)))
    A = -(dv^2)/(s^2);
else
    A = inf;
end

accel = min(A,a_perf);

end

