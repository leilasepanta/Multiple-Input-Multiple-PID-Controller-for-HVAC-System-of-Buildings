%%  HVAC system for a 2-stories office building with 2 rooms in each floor
% clc;
% close all;
clear all;

% simulation approach two: variable step solver between control updates
Te = 298.15; [298.15, 297.15, 296.15, 295.15 ,298.15, 297.15, 296.15, 295.15,298.15, 297.15]; %desired temperature of each zone
Tf = 60*60*24; %total simulation time (s)
Tu = 120;10; %control update period (s)

%% plant (system) variables
Ns = 10; %number of states

R1= 30000;  %%resistiors(ohms)
R2=20000;
R3=20000;
R4=20000;
R5=20000;
R6=56000;
R11=30000;
R22=30000;
R33=30000;
R44=45000;

R12=7000;
R23=7000;
R34=7000;
R45=7000;
R56=7000;
R1122=7000;
R2233=7000;
R3344=7000;
R611=25000;
R622=25000;
R633=25000;
R533=25000;
R444=26000;
R344=26000;
R244=26000;
R144=26000;


C1=5;
C2=1;
C3=1;
C4=1;
C5=1;
C6=7;
C11=2;
C22=2;
C33=3;
C44=7;

% c=0.001;  %%capacitors(farads)


%% Defining A, B, C, and D matrices for state space representation



 A=[(-1/C1)*((1/R144)+(1/R1)+(1/R12)),(1/C1)*(1/R12),0,0,0,0,0,0,0,(1/C1)*(1/R144);
     (1/C2)*(1/R12), (-1/C2)*((1/R12)+(1/R244)+(1/R2)+(1/R23)),(1/C2)*(1/R23),0,0,0,0,0,0,(1/C2)*(1/R244);
     0,(1/(C3*R23)),(-1/C3)*(1/R3+1/R23+1/R344+1/R34),(1/(C3*R34)),0,0,0,0,0,(1/C3)*(1/R344);
     0,0,(1/C4)*(1/R34),(-1/C4)*(1/R4+1/R45+1/R34+1/R444),(1/(C4*R45)),0,0,0,0,(1/C4)*(1/R444);
     0,0,0,(1/C5)*(1/R45),(-1/C5)*(1/R5+1/R45+1/R56+1/R533),(1/C5)*(1/R56),0,0,(1/C5)*(1/R533),0;
     0,0,0,0,(1/C6)*(1/R56),(-1/C6)*(1/R6+1/R56+1/R633+1/R622+1/R611),(1/(C6*R611)),(1/(C6*R622)),(1/(C6*R633)),0;
     0,0,0,0,0,(1/C11)*(1/R611),(-1/C11)*(1/R611+1/R1122+1/R11),(1/C11)*(1/R1122),0,0;
     0,0,0,0,0,(1/C22)*(1/R622),(1/C22)*(1/R1122),(-1/C22)*(1/R622+1/R1122+1/R2233+1/R22),(1/C22)*(1/R2233),0;
     0,0,0,0,(1/C33)*(1/R533),(1/C33)*(1/R633),0,(1/C33)*(1/R2233),(-1/C33)*(1/R633+1/R533+1/R2233+1/R3344+1/R33),(1/C33)*(1/R3344);
     (1/C44)*(1/R144),(1/C44)*(1/R244),(1/C44)*(1/R344),(1/C44)*(1/R444),0,0,0,0,(1/C44)*(1/R3344),(-1/C44)*(1/R144+1/R244+1/R344+1/R444+1/R3344+1/R44)];

 
 
 B=[1/C1,0,0,0,0,0,0,0,0,0;
     0,1/C2,0,0,0,0,0,0,0,0;
     0,0,1/C3,0,0,0,0,0,0,0;
     0,0,0,1/C4,0,0,0,0,0,0;
     0,0,0,0,1/C5,0,0,0,0,0;
     0,0,0,0,0,1/C6,0,0,0,0;
     0,0,0,0,0,0,1/C11,0,0,0;
     0,0,0,0,0,0,0,1/C22,0,0;
     0,0,0,0,0,0,0,0,1/C33,0;
     0,0,0,0,0,0,0,0,0,1/C44];

 
 H=[1/C1,0,0,0,0,0,0,0,0,0,1/(R1*C1),0,0,0,0,0,0,0,0,0;
     0,1/C2,0,0,0,0,0,0,0,0,0,1/(R2*C2),0,0,0,0,0,0,0,0;
     0,0,1/C3,0,0,0,0,0,0,0,0,0,1/(R3*C3),0,0,0,0,0,0,0;
     0,0,0,1/C4,0,0,0,0,0,0,0,0,0,1/(R4*C4),0,0,0,0,0,0;
     0,0,0,0,1/C5,0,0,0,0,0,0,0,0,0,1/(R5*C5),0,0,0,0,0;
     0,0,0,0,0,1/C6,0,0,0,0,0,0,0,0,0,1/(R6*C6),0,0,0,0;
     0,0,0,0,0,0,1/C11,0,0,0,0,0,0,0,0,0,1/(R11*C11),0,0,0;
     0,0,0,0,0,0,0,1/C22,0,0,0,0,0,0,0,0,0,1/(R22*C22),0,0;
     0,0,0,0,0,0,0,0,1/C33,0,0,0,0,0,0,0,0,0,1/(R33*C33),0;
     0,0,0,0,0,0,0,0,0,1/C44,0,0,0,0,0,0,0,0,0,1/(R44*C44)];
 
 C=eye(10);
 
 D= zeros(size(B));
 
 


sys = struct('A',A, ...
    'B', B, ...
    'C', C, ...
    'H', H);

t = 0:Tu:Tf; %times at which we sample system state and update control input
x = zeros(length(t),Ns); %system state at sampling times
x(1,:) = Te*ones(1,Ns); %initial state of system

%% controller variables
Nu = 10; %number of inputs to system
u = zeros(Nu,1); %initial control input

% PI controller gains for HVAC output
kp1 = 0.0283; kp2 = 0.00706; kp3 = 0.00397; kp4 = 0.00381; kp9 = 0.00878;
ki1 = 0.000899; ki2 = 2.95e-05; ki3 = 0.000164; ki4 = 0.000138; ki9 = 2.45e-05;
kp5 = 0.00713; kp6 = 0.0465; kp7 = 0.0137; kp8 = 0.0164; kp10 = 0.0408;
ki5 = 0.0026; ki6 = 0.000654; ki7 = 0.00152; ki8 = 0.00126; ki10 = 0.00385;
kp = [kp1;kp2;kp3;kp4;kp5;kp6;kp7;kp8;kp9;kp10];
ki = [ki1;ki2;ki3;ki4;ki5;ki6;ki7;ki8;ki9;ki10];

p = 0; %proportional term of PI controller
pint = 0; %integral term of PI controller

% derivative gains for decoupling controller


Der=[ 1.001 ,  3.841e-05 , -1.912e-06 , -1.461e-07 , -2.035e-07  ,  1.139e-07 ,  2.308e-07 , -4.059e-07 ,  2.434e-06 ,  1.601e-06 ; ...
    1.778e-05   ,    1.002 ,  9.252e-05 ,  2.588e-07 ,  1.492e-06  , -4.982e-07 , -3.887e-07 ,  7.442e-07 , -5.222e-07 , -6.692e-06 ; ...
    5.933e-07 ,  0.0001211   ,        1 , -6.162e-07 ,  1.326e-06  ,  -2.595e-07 , -1.854e-07 ,  3.071e-07 ,  1.555e-05 ,  1.459e-06 ; ...
     -9.724e-07 ,  2.477e-07 , -1.623e-06   ,        1 , -3.369e-07 ,  1.813e-07 ,   2.81e-07 , -1.472e-06 , -0.0002163 ,  1.984e-06  ; ...
     -7.934e-08 ,  5.937e-07 ,  7.678e-07 , -1.273e-06 ,          1,  6.784e-06 , -1.841e-08 , -2.627e-05 , -0.0005885 ,  -3.14e-06  ; ...
     2.22e-07 , -5.789e-08  , 1.302e-07  ,   8.7e-07 ,  6.419e-06 ,         1 ,  2.232e-06 ,  1.249e-05 ,  0.0004394 ,  1.478e-06  ; ...
     -6.089e-08 , -5.467e-08 , -6.508e-08  ,  4.44e-08 , -1.982e-07  ,  2.378e-06   ,        1 ,  2.121e-06 ,  8.694e-05 ,  2.872e-07 ; ...
     3.983e-07 , -2.933e-08 ,  2.537e-07 ,  2.745e-06 , -1.248e-05  ,   1.13e-05 ,  4.236e-07    ,       1  ,    0.0192 ,  3.055e-05  ; ...
     1.759e-06 , -5.278e-07 , -9.881e-05  ,   0.00136 ,  -0.002485  ,  0.0004257 ,  7.247e-06  ,   0.01891    ,   1.041  ,  0.002176  ; ...
     3.338e-06  , -4.28e-05  , 1.495e-08 ,  2.422e-06 , -6.196e-06  ,   2.999e-06 ,  9.714e-08 ,  7.382e-05  ,  0.003754   ,        1];

%% evaluate plant response
tic;
for i = 2:length(t)
    % find plant response over previous sampling period
    f_w = @(T,Y)plant(T,Y,sys,u); %handle for ode solver (function it evaluates)
    
    [~,xds] = ode23(f_w,t(i-1:i),x(i-1,:)');
    x(i,:) = xds(end,:);
    
    % update control input for next sampling period
    e = (Te - x(i,:))'; %desired vs. actual
    
    p = Der*e; %p(t)
    
    % calculate integral for PI controller: use discretized version from SATS project
    dt = t(i) - t(i-1);
    pint = pint + p*dt; %integral(p,0,t)
    
    % find control input
    u = kp.*p + ki.*pint;
    umax = 1e-3; umin = -1e-3;
    ind_umax = (u > umax); u(ind_umax) = umax;
    ind_umin = (u < umin); u(ind_umin) = umin;
    
%     u = zeros(Ns,1);
%     disp(u);
%     null = input('...');
end
toc;
% x=G*u+K*dist;


figure;plot(t,x);hold on;plot(t,5*sin(2*pi*t/86400)+298.15);
xlabel('time [s]');ylabel('Temperature [K]');
%title('Sine wave Response for the electrical model of an office building');
legend('1','2','3','4','5','6','7','8','9','10','outside');
