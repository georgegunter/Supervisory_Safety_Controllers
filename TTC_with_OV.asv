function [accel] = TTC_with_OV(s,v,dv,p)

k_OV = p(1);
OV = p(2);

OV_accel = k_OV*(OV - v);

tau=p(3);

A = 0;
if(dv>=(1/(tau*s)))
    A = -(dv^2)/(s);
else
    A = inf;
end

accel = min(A,OV_accel);


end

