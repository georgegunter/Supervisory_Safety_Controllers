function [p_follower,v_follower,p_leader,v_leader,time] = ...
    emergency_brake_scenario(performance_controller,safety_controller,v0,sim_length)
% Author: George Gunter (2022)

% This function simulations a controller approach a stopped vehicle which
% is 300 meters away at the beginning of the sim. Need to specify an
% accel_controller of the form f(s,v,dv), an initial speed v0, and the length
% of simulation sim_length.

dt = 0.1; %time step
time = 0:dt:sim_length; % times
sim_steps = length(time); % How many steps are taken

a_lead_decel = -2.8;

max_possible_decel = -3.0;
time_to_stop = -(v0/max_possible_decel);
p0 = (1/2)*(max_possible_decel*time_to_stop^2)+v0*time_to_stop;

p_leader = zeros(sim_steps,1); %positions
v_leader = zeros(sim_steps,1); %speeds

p_leader(1) = p0;
v_leader(1) = v0;

for i=2:sim_steps
    v_new = v_leader(i-1) + a_lead_decel*dt;
    v_new = max(0,v_new); %Speed doesn't go below 0
    v_leader(i) = v_new;
    p_new = p_leader(i-1) + v_leader(i-1)*dt;
    p_leader(i) = p_new;
     
end
    
% Run this corresponding simulation: 
[p_follower,v_follower] = sim_follower(performance_controller,safety_controller,v0,p_leader,v_leader,sim_steps,dt);


end