clear
clc
close all
% this script defines parameters
% declare all variables
payload.m = 2.28; % kg, mass of payload
payload.l = 0.25; % m, length of payload
payload.w = 0.14; % m, width of payload
payload.Izz = (1/12)*payload.m*(payload.l^2 + payload.w^2); % kg*m^2, mass moment of inertia about z
payload.Ft = 8; % N, force of thrust from EDF

payload.fin_l = 0.1; % m, length of fin
payload.fin_w = 0.025; % m, width of fin

% time specifications
payload.dt = 0.001;
payload.tFinal = 10;

% initial/desired states
payload.theta0 = -30; % initial heading of payload
payload.dtheta0 = 0; % initial angular velocity of payload
payload.phi0 = 0; % initial fin angle
payload.theta_des = 0; % desired heading is 0

% plotting
payload.plotbounds = [-.5 .5 -.5 .5];

% controller gains
payload.kp = 1.7;
payload.ki = 1;
payload.kd = 0.86;

% rewrite variables
F = payload.Ft;
I = payload.Izz;
d = payload.l/2;
kp = payload.kp;
ki = payload.ki;
kd = payload.kd;

%% Root Locus Design

% uncontrolled system
G = tf(F*d,[I 0 0]);
subplot(2,1,1)
step(G)
title("Step Response - Uncontrolled System")
subplot(2,1,2)
rlocus(G)

 
% add PID controller, from sisotools() w/ minimizing response time
K = 0.5;
H = tf([kd kp ki],[1 0]);
% model closed loop function
figure();
subplot(2,1,1)
Gcl = feedback(K*H*G,1);
step(Gcl)
title("Step Response - Controlled System")
grid on

subplot(2,1,2)
rlocus(K*H*G)


% ode45 setup   
ic = [payload.theta0 payload.dtheta0];
tSpan = [0 payload.tFinal];

writerObj = VideoWriter('Payload Animation theta=30', 'MPEG-4');
open(writerObj);
fig = figure();

[t,x] = ode45(@(t, x) controlled_ode(t, x, payload, fig, writerObj), tSpan, ic);
theta = x(:,1);

close(writerObj);

figure()

plot(t,theta)
xlabel("Time (s)")
ylabel("\Theta (\circ)")
title("Time Response of System, \Theta_i = -30\circ")
grid on

%%
%
% <include>controlled_ode</include>
%

%%
%
% <include>drawPayload</include>
%