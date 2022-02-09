%% Brief demonstration on how to run sim_leader and sim_follower:
%Author: George Gunter (2021/8/12)

sim_steps = 400;
dt = 0.05;

% Random acceleration controller (just brakes constantly)
% accel_controller = @(s,v,dv) -1;
accel_controller = @(s,v,dv) 0.5*(s - 15);
% accel_controller = @(s,v,dv) 0.5*(1.0 - v); 

% Simulate leader trajectory:
a_lead = 0.0;
v0_lead = 1.0;
s0_lead = 20.0;
[p_leader,v_leader] = sim_leader(s0_lead,v0_lead,a_lead,sim_steps,dt);

% Simulate follower trajectory:
v0_follower = 10.0;
[p_follower,v_follower] = sim_follower(v0_follower,accel_controller,p_leader,v_leader,sim_steps,dt);

disp('Simulation finished')
%% Plot results:

spacing = p_leader - p_follower;

time = linspace(0,sim_steps*dt,sim_steps);
figure()
plot(time,spacing,'LineWidth',5)
ylabel('Spacing [m]','FontSize',20)
xlabel('Time [s]','FontSize',20)


