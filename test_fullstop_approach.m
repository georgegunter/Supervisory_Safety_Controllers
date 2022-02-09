%% Run simulation for CBF managed OV: 

p = [0.1,2.0,0.5,0.5,15.0]; %[k_t,t_min,k_1,k_2,s_min]
safety_controller = @(a_perf,s,v,dv) CBF_time_gap_space_gap(a_perf,s,v,dv,p);

performance_controller = @(s,v,dv) 0.1*(15.0 - v);

v0 = 15.0;

sim_length = 30;

[p_follower,v_follower,p_leader,v_leader,time] = ...
    emergency_brake_scenario(performance_controller,safety_controller,v0,sim_length);
clc;
disp('Simulation complete')
disp('Minimum spacing gap: ')
disp(min(p_leader-p_follower))

%% Plot results:

space_gap = p_leader - p_follower;
subplot(3,1,1)
plot(time,space_gap,'linewidth',3)
ylabel('Spacing gap [m]','fontsize',20)
title('CBF managing OV: Constant decel','fontsize',20)
grid on;
subplot(3,1,2)
hold on
plot(time,v_follower,'linewidth',3)
plot(time,v_leader,'linewidth',3)
legend('Ego','Leader')
ylabel('Speed [m/s]','fontsize',20)
xlabel('Time [s]','fontsize',20)
grid on;
subplot(3,1,3)
plot(time(2:end),diff(v_follower)/.1,'linewidth',3)
ylabel('Acceleration [m/s^2]','fontsize',20)
xlabel('Time [s]','fontsize',20)
grid on;


