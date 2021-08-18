addpath(genpath('src'));
close all; clear; clc;
%% set number of links
mdl = Model(8);

%% settings
mdl = mdl.set('Tsim',10);
mdl = mdl.setElements(40);
mdl = mdl.setFrequency(30);
mdl = mdl.setLength(0.025);
mdl.m0(end) = 0.5;
mdl.r0(end) = 0.005;

%% offset parameters
%mdl.tau = @(mdl) Controller(mdl);

%% simulate with initial conditions
mdl.q0(2:3:end) = 1;
mdl = mdl.simulate;

%% post-processing data
t  = mdl.t;
l0 = mdl.get('l0');
l  = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx = mdl.q(:,2:3:3*mdl.Nlink);
ky = mdl.q(:,3:3:3*mdl.Nlink);

f = figure(101); f.Name = 'Bishop parameters';
subplot(3,1,1); set(gca,'linewidth',1.5);
plot(t,l*1e3,'linewidth',1.5); hold on;
ylabel('$l$ (mm)','interpreter','latex','fontsize',12);
axis([0 10 25 30]); grid on;
subplot(3,1,2); set(gca,'linewidth',1.5);
plot(t,kx,'linewidth',1.5); hold on;
ylabel('$\kappa_x$ (m$^{-1}$)','interpreter','latex','fontsize',12); 
axis([0 10 -80 80]); grid on;
subplot(3,1,3); set(gca,'linewidth',1.5);
plot(t,ky,'linewidth',1.5); hold on;
ylabel('$\kappa_y$ (m$^{-1}$)','interpreter','latex','fontsize',12); 
xlabel('time (s)','interpreter','latex','fontsize',12);
axis([0 10 -80 80]); grid on;

%% play animation soft robot
f = figure(103);
Q = mdl.q;

for ii = 1:fps(mdl.t,75):length(mdl.t)
    figure(103); cla;
    mdl.show(Q(ii,:),greycolors(4));
    groundplane(0.02);
    axis(0.4*[-1 1 -1 1 -0.75 1.1]);
    view(0,0); drawnow;
    f.Name = [' Time =',num2str(mdl.t(ii),3)];
    drawnow();
    grid on;
end

%% model-based controller
function tau = Controller(mdl)
P = mdl.Phi; p = mdl.p;

Ad = adjointSE3inv(P,p);

J     = mdl.J;
f     = [0;0;0;0;0;-0.5];
delta = J.'*(Ad*f);
tau   = 0*delta;
end

    