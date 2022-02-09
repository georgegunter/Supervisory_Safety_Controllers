%% Run simulation for CBF managed OV: 

p = [0.5,15.0,0.1,2.0,0.5,0.5,15.0]; %[k_OV,OV,k_t,t_min,k_1,k_2,s_min]
accel_controller = @(s,v,dv) CBF_with_OV(s,v,dv,p);

v0 = 30.0;

sim_length = 80;

[p_follower,v_follower,p_leader,v_leader,time] = ...
    fullstop_approach_scenario(accel_controller,v0,sim_length);

disp('Simulation complete')
disp('Minimum spacing gap: ')
disp(min(p_leader-p_follower))

%% Plot results:

space_gap = p_leader - p_follower;
subplot(2,1,1)
plot(time,space_gap,'linewidth',3)
ylabel('Spacing gap [m]','fontsize',20)
title('CBF managing OV','fontsize',20)
grid on;
subplot(2,1,2)
plot(time,v_follower,'linewidth',3)
ylabel('Speed [m/s]','fontsize',20)
xlabel('Time [s]','fontsize',20)
grid on;

%% Run simulation for TTC: 

% p = [0.1,30.0,0.1,2.0,0.5,0.5,15.0]; %[k_OV,OV,k_t,t_min,k_1,k_2,s_min]
% accel_controller = @(s,v,dv) CBF_with_OV(s,v,dv,p);





v0 = 30.0;

sim_length = 80;

[p_follower,v_follower,p_leader,v_leader,time] = ...
    fullstop_approach_scenario(accel_controller,v0,sim_length);

disp('Simulation complete')
disp('Minimum spacing gap: ')
disp(min(p_leader-p_follower))


