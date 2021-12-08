%Author: George Gunter (2021/8/12)

close all
clear all
%leaders accleration
ul=@(t) -1+0*t; 
%time horizon
T=10;
%number of time steps
nt=100;
%time grid
t=linspace(0,T,nt);
%initial position leader
xl0=0;
%initial velocity leader
vl0=10;
%Computing leader's velocity and trajectory based on its given acceleration
%ul and initial velocity vl0 and position xl0 on the time horizon T
[~,vl]=ode45(@(t,y)ul(t),t,vl0);
vl=@(x)interp1(t, vl, x);
[~,xl]=ode45(@(t,y)vl(t),t,xl0);
xl=@(x)interp1(t, xl, x);

%initial position follower
x0=-5;
%initial velocity follower
v0=10;
%maximum braking
u_min=-5;
%car length
l=4.5;

%Initializing followers acceleration u by assuming that we break as heavily
%as the leader to avoid collision for the initialization phase
u=u_min*ones(1,nt);

%fmincon options
options = optimoptions('fmincon','Display','iter-detailed', ...
                                    'maxfunevals',5e2, 'StepTolerance',1e-10,'algorithm', 'interior-point');

u_opt=fmincon(@(z)fun(T,nt,t,xl,vl,v0,x0,l,z),u,[],[],[],[],u_min*ones(1,nt),[],[],options);

%interpolate optimized acceleration
u_opt=@(x)interp1(t, u_opt, x);
%compute corresponding velocity
 [~,v_opt]=ode45(@(t,y)u_opt(t),t,v0);
 v_opt=@(x)interp1(t, v_opt, x);
 %compute corresponding position
 [~,x_opt]=ode45(@(t,y)v_opt(t),t,x0);
 x_opt=@(x)interp1(t, x_opt, x);
figure(1)
plot(t,vl(t),t,v_opt(t))
legend('Velocity leader', 'Velocity follower')
figure(2)
plot(t,xl(t),t,x_opt(t))
legend('Position leader', 'Position follower')
%define the objective function
function y=fun(T,nt,t,xl,vl,v0,x0,l,u)
    %compute for a given acceleration profile u the position and velocity
    %of the follows
    u=@(x)interp1(t, u, x);
    [~,v]=ode45(@(t,y)u(t),t,v0);
    v=@(x)interp1(t, v, x);
    [~,x]=ode45(@(t,y)v(t),t,x0);
    x=@(z)interp1(t, x, z);
    %compute the cost function which here consists of a barrier function
    %penalizing if cars get closer than 1 unit. The outer max is to avoid
    %the objective function being undefined. Could also have some
    %steering/tracking term for the velocity, but this could be refined
    %dependent on what we want.
     y=-T/nt*sum(log(max(1e-12,min(xl(t)-x(t)-l,1))));
end

