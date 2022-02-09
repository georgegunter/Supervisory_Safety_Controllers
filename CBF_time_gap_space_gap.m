function [accel] = CBF_time_gap_space_gap(a_perf,s,v,dv,p)
%CBF_WITH_OV Summary of this function goes here
%   Detailed explanation goes here

k_t = p(1);%0.1;
t_min = p(2);%2.0;
k1 = p(3);%0.5;
k2 = p(4);%0.5;
s_min = p(5);%15.0;

max_accel_time_gap = (1/t_min)*(dv) + (k_t/t_min)*(s - t_min*v);

max_accel_space_gap = (k1+k2)*dv + (k1*k2)*(s-s_min);

accel = min([a_perf,max_accel_time_gap]);
accel = min([accel,max_accel_space_gap]);

end

