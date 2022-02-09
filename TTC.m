function [accel] = TTC(a_perf,s,v,dv,p)

tau=p(1);

A = 0;
if(dv>=(1/(tau*s)))
    A = -(dv^2)/(s);
else
    A = inf;
end

accel = min(A,a_perf);

end

