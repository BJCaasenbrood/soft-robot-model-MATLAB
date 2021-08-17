addpath(genpath('src'));
close all; clear; clc;
%% set number of links
mdl = Model(8);

%% settings
mdl = mdl.set('Tsim',30,'Adaptive',true);
mdl = mdl.setElements(40);
mdl = mdl.setFrequency(125);
mdl = mdl.setLength(0.025);

%% offset parameters
mdl.Pihat(1) = mdl.Pi(1)*0.75;
mdl.Pihat(2) = mdl.Pi(2)*0.75;
mdl.Pihat(4) = mdl.Pi(4)*0.55;
mdl.Pihat(5) = mdl.Pi(5)*0.55;

mdl.tau       = @(mdl) Controller(mdl);
mdl.updatelaw = @(mdl) UpdateLaw(mdl,1);

%% simulate with initial conditions
mdl.q0(2) = 1;
mdl = mdl.simulate;

%% post-processing data
t  = mdl.t;
l0 = mdl.get('l0');
l  = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx = mdl.q(:,2:3:3*mdl.Nlink);
ky = mdl.q(:,3:3:3*mdl.Nlink);
dkx = mdl.dq(:,2:3:3*mdl.Nlink);
dky = mdl.dq(:,2:3:3*mdl.Nlink);

ld  = mean(l0)*(1+0.01*sin(t) + 0.01);
kxd = 40*sin(t);
kyd = 40*cos(t);

f = figure(101); f.Name = 'Bishop parameters';
subplot(3,1,1); set(gca,'linewidth',1.5);
plot(t,l*1e3,'Color',greycolors(4),'linewidth',2.5); hold on;
plot(t,ld*1e3,'--','Color',greycolors(4),'linewidth',2.5); 
ylabel('$l$ (mm)','interpreter','latex','fontsize',19);
axis([0 40 64.5 66.5]); grid on;
subplot(3,1,2); set(gca,'linewidth',1.5);
plot(t,kx,'Color',greycolors(4),'linewidth',2.5); hold on;
plot(t,kxd,'--','Color',greycolors(4),'linewidth',2.5); 
ylabel('$\kappa_x$ (m$^{-1}$)','interpreter','latex','fontsize',19); 
axis([0 40 -50 50]); grid on;
subplot(3,1,3); set(gca,'linewidth',1.5);
plot(t,ky,'Color',greycolors(4),'linewidth',2.5); hold on;
plot(t,kyd,'--','Color',greycolors(4),'linewidth',2.5); 
ylabel('$\kappa_y$ (m$^{-1}$)','interpreter','latex','fontsize',19); 
xlabel('time (s)','interpreter','latex','fontsize',19);
axis([0 40 -50 50]); grid on;

%% plotting adaptive stiffness
% figure(102);
% ke = []; ke_ = [];
% kb = []; kb_ = [];
% 
% for ii = 1:length(mdl.t)
%     [Ke, Kb] = nonlinearStiffness(mdl,mdl.q(ii,:),mdl.Pi);
%     [Ke_, Kb_] = nonlinearStiffness(mdl,mdl.q(ii,:),mdl.Pihat(ii,:));
%     
%     ke = [ke;Ke]; ke_ = [ke_;Ke_];
%     kb = [kb;Kb]; kb_ = [kb_;Kb_];
% end
% 
% plot(t,ke./ke_,'Color',greycolors(2),'linewidth',2.5); hold on;
% plot(t,kb./kb_,'Color',greycolors(4),'linewidth',2.5); 
% ylabel('Evolution of stiffness estimate','interpreter','latex','fontsize',19); 
% xlabel('time (s)','interpreter','latex','fontsize',19);
% set(gca,'linewidth',1.5); axis([0 40 0.75 2]); grid on;
% legend('$\frac{k_e(q)}{\hat{k}_e(q,\Pi)}$ \vspace{15mm}',...
%     '$\frac{k_b(q)}{\hat{k}_b(q,\Pi)}$','interpreter','latex',...
%     'fontsize',27,'orientation','horizontal');
error('_');

%% play animation soft robot
f = figure(103);
Q = mdl.q;

for ii = 1:fps(mdl.t,75):length(mdl.t)
    figure(103); cla;
    mdl.show(Q(ii,:),greycolors(4));
    groundplane(0.02);
    axis equal; %axis([-0.05 0.05 -0.05 0.05 0 0.07]);
    f.Name = [' Time =',num2str(mdl.t(ii),3)];
    drawnow();
    grid on;
end

%% model-based controller
function tau = Controller(mdl)
t = mdl.t;
P = mdl.Phi;
p = mdl.p;

Ad = adjointSE3inv(P,p);

J     = mdl.J;
f     = [0;0;0;0;0;-25];
delta = J.'*(Ad*f);

Kp_ = diag([5, 0.001, 0.001]);
Kd_ = diag([1, 0.001, 0.001]);

Lambda = eye(3*mdl.Nlink);
qd_     = [0.01*sin(t) + 0.01; 20*sin(t); 20*cos(t)];
dqd_    = [0.01*cos(t); 20*cos(t); -20*sin(t)];
ddqd_   = [-0.01*sin(t); -20*sin(t); -20*cos(t)];

qd = kron(ones(mdl.Nlink,1),qd_);
dqd = kron(ones(mdl.Nlink,1),dqd_);
ddqd = kron(ones(mdl.Nlink,1),ddqd_);
Kp = kron(eye(mdl.Nlink),Kp_);
Kd = kron(eye(mdl.Nlink),Kd_);

e    = mdl.q - qd;
de   = mdl.dq - dqd;
er   = de + Lambda*e;

dqr  = dqd  - Lambda*e;
ddqr = ddqd - Lambda*de;

tau  = mdl.M*ddqr + mdl.C*dqr + mdl.G + mdl.K*mdl.q - Kp*e - Kd*er ...
+ delta;
end

function [dp,e] = UpdateLaw(mdl,alpha)
t  = mdl.t;
Y  = mdl.Y;
Ge = 1e3;
Gb = 3e-2;

Gamma  = alpha*diag([Ge,Ge,0,Gb,Gb,0,0]);

qd_  = [0.01*sin(t) + 0.01; 20*sin(t); 20*cos(t)];
dqd_ = [0.01*cos(t); 20*cos(t); -20*sin(t)];

qd   = kron(ones(mdl.Nlink,1),qd_);
dqd  = kron(ones(mdl.Nlink,1),dqd_);

e  = mdl.q - qd;
de = mdl.dq - dqd;
er = de + e;

dp = -Gamma*Y.'*er;
end

function [Ke, Kb] = nonlinearStiffness(Model,x,Pi)
parL = Model.get('l0'); 
Q    = x(:);  
eps  = parL*Q(1);
beta = parL*(sqrt(Q(2)^2 + Q(3)^2));
phi  = atan2(Q(3),Q(2));

w = 1/(2*pi/6);
faxi = @(x,a)-(0.5*cos(w*pi*(x) + pi/2)+0.5)*a+(1+a);

alpha = (numel(x)/3);
Fax = faxi(phi,Model.Pihat(7)*beta);
Ke = alpha*(Pi(1) + Pi(2)*(tanh(Pi(3)*eps)^2 - 1));
Kb = alpha*Fax*(Pi(4) + Pi(5)*(tanh(Pi(6)*beta)^2 - 1));    
end
    