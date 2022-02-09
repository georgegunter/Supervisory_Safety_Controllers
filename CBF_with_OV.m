function [accel] = CBF_with_OV(s,v,dv,p)
%CBF_WITH_OV Summary of this function goes here
%   Detailed explanation goes here

k_OV = p(1);
OV = p(2);

OV_accel = k_OV*(OV - v); %Tries to drive at a constant speed

k_t = p(3);%0.1;
t_min = p(4);%2.0;
k1 = p(5);%0.5;
k2 = p(6);%0.5;
s_min = p(7);%15.0;

max_accel_time_gap = (1/t_min)*(dv) + (k_t/t_min)*(s - t_min*v);

max_accel_space_gap = (k1+k2)*dv + (k1*k2)*(s-s_min);

accel = min([OV_accel,max_accel_time_gap]);
accel = min([accel,max_accel_space_gap]);

end

