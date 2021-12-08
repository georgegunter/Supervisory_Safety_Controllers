function [p_leader,v_leader] = sim_leader(p0,v0,a,sim_steps,dt)
%Author: George Gunter (2021/8/12)

p_leader = zeros(sim_steps,1);
v_leader = zeros(sim_steps,1);

%Set initial conditions:
p_leader(1) = p0;
v_leader(1) = v0;

for i=2:sim_steps
    dv_dt = a;
    dp_dt = v_leader(i-1);
    
    p_leader(i) = p_leader(i-1) + dp_dt*dt;
    v_leader(i) = v_leader(i-1) + dv_dt*dt;
    
    if(v_leader(i) < 0.0)
        v_leader(i) = 0.0;
    end
    
end
    
end

