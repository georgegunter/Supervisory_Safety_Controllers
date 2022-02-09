function [p_follower,v_follower] = sim_follower(v0,accel_controller,p_leader,v_leader,sim_steps,dt)
%Author: George Gunter (2021/8/12)

p_follower = zeros(sim_steps,1);
v_follower = zeros(sim_steps,1);

% Set initial conditions:
p_follower(1) = 0;
v_follower(1) = v0;


for i=2:sim_steps
    
    s = p_leader(i-1) - p_follower(i-1);    
    
    v = v_follower(i-1);
    dv = v_leader(i-1) - v_follower(i-1);
    
    dv_dt = accel_controller(s,v,dv);
    %Clipping:
    dv_dt = max([-3,dv_dt]); %Max braking
    dv_dt = min([1.5,dv_dt]); %Max accel
    
    dp_dt = v_follower(i-1);
    
    p_follower(i) = p_follower(i-1) + dp_dt*dt;
    v_follower(i) = v_follower(i-1) + dv_dt*dt;
    
    % Don't allow for negative speeds:
    if(v_follower(i) < 0.0)
        v_follower(i) = 0.0;
    end    
end

end

