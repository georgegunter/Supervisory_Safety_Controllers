function [p_follower,v_follower,p_leader,v_leader,time] = fullstop_approach_scenario(accel_controller,v0,sim_length)
% Author: George Gunter (2022)

% This function simulations a controller approach a stopped vehicle which
% is 300 meters away at the beginning of the sim. Need to specify an
% accel_controller of the form f(s,v,dv), an initial speed v0, and the length
% of simulation sim_length.

dt = 0.1; %time step
time = 0:dt:sim_length; % times
sim_steps = length(time); % How many steps are taken

% Make positions and speeds for leading vehicle which is stopped.
p_leader = ones(sim_steps,1)*300; %positions
v_leader = zeros(sim_steps,1); %speeds

% Run this corresponding simulation: 
[p_follower,v_follower] = sim_follower(v0,accel_controller,p_leader,v_leader,sim_steps,dt);


end

