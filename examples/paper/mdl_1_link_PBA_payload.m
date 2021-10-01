addpath(genpath('src'));
close all; clear; clc;
%% set number of links
mdl = Model(1);

%% settings
mdl = mdl.set('Tsim',50,'Adaptive',true);
mdl = mdl.setElements(5);
mdl = mdl.setFrequency(60);
mdl = mdl.setLength(0.065);
mdl = mdl.setLoad(0.175);

%% offset parameters
mdl.Pihat(1) = mdl.Pi(1)*0.75;
mdl.Pihat(2) = mdl.Pi(2)*0.75;
mdl.Pihat(4) = mdl.Pi(4)*0.55;
mdl.Pihat(5) = mdl.Pi(5)*0.55;
mdl.Pihat(8) = mdl.Pi(8)*0.65;

mdl.tau       = @(mdl) Controller(mdl);
mdl.updatelaw = @(mdl) UpdateLaw(mdl,1);

%% simulate with initial conditions
mdl = mdl.simulate;

%% post-processing data
t  = mdl.t;
l0 = mdl.get('l0');
e = mdl.q(:,1:3:3*mdl.Nlink);
kx = mdl.q(:,2:3:3*mdl.Nlink);
ky = mdl.q(:,3:3:3*mdl.Nlink);
dkx = mdl.dq(:,2:3:3*mdl.Nlink);
dky = mdl.dq(:,2:3:3*mdl.Nlink);

ed  = 0.01*sin(t) + 0.01;
kxd = 30*sin(t);
kyd = 30*cos(t);

f = figure(101); f.Name = 'Bishop parameters';
subplot(3,1,1); set(gca,'linewidth',1.5);
plot(t,e,'Color',gcol(4),'linewidth',2.5); hold on;
plot(t,ed,'--','Color',gcol(4),'linewidth',2.5); 
ylabel('$\varepsilon$ (-)','interpreter','latex','fontsize',18);
grid on; axis([0 mdl.t(end) -5e-3 25e-3]);
subplot(3,1,2); 
plot(t,kx,'Color',gcol(4),'linewidth',2.5); hold on;
plot(t,kxd,'--','Color',gcol(4),'linewidth',2.5); 
set(gca,'linewidth',1.5);
ylabel('$\kappa_x$ (m$^{-1}$)','interpreter','latex','fontsize',18); 
axis([0 mdl.t(end) -50 50]); grid on;
subplot(3,1,3); set(gca,'linewidth',1.5);
plot(t,ky,'Color',gcol(4),'linewidth',2.5); hold on;
plot(t,kyd,'--','Color',gcol(4),'linewidth',2.5); 
ylabel('$\kappa_y$ (m$^{-1}$)','interpreter','latex','fontsize',18); 
xlabel('time (s)','interpreter','latex','fontsize',18);
axis([0 mdl.t(end) -50 50]); grid on;

%% plotting adaptive stiffness
figure(102);
ke = []; ke_ = [];
kb = []; kb_ = [];

for ii = 1:length(mdl.t)
    [Ke, Kb] = nonlinearStiffness(mdl,mdl.q(ii,:),mdl.Pi);
    [Ke_, Kb_] = nonlinearStiffness(mdl,mdl.q(ii,:),mdl.Pihat(ii,:));
    
    ke = [ke;Ke]; ke_ = [ke_;Ke_];
    kb = [kb;Kb]; kb_ = [kb_;Kb_];
end

plot(t,ke./ke_,'Color',gcol(2),'linewidth',2.5); hold on;
plot(t,kb./kb_,'Color',gcol(3),'linewidth',2.5); 
plot(t,mdl.Pi(8)./mdl.Pihat(:,8),'Color',gcol(4),'linewidth',2.5,'linestyle','-.'); 
ylabel('Evolution of stiffness estimate','interpreter','latex','fontsize',19); 
xlabel('time (s)','interpreter','latex','fontsize',19);
set(gca,'linewidth',1.5); axis([0 mdl.t(end) 0.75 2]); grid on;
legend('$\frac{k_e(q)}{\hat{k}_e(q,\Pi)}$ \vspace{15mm}',...
    '$\frac{k_b(q)}{\hat{k}_b(q,\Pi)}$','$\frac{m_\delta}{\hat{m}_\delta(\Pi)}$','interpreter','latex',...
    'fontsize',27,'orientation','horizontal');
% error('_');

%% recover trajectory end effector
P = zeros(length(mdl.t),3);
for ii = 1:length(mdl.t)
    [p0, ~] = mdl.computeEndEffector(mdl.q(ii,:),mdl.dq(ii,:));
    P(ii,:) = p0.';
end

%% animate soft robot
for ii = 1:fps(mdl.t,20):length(mdl.t)
    figure(103); cla;
    
    mdl.show(mdl.q(ii,:),gcol(3));
    
    plot3(P(1:ii,1),P(1:ii,2),P(1:ii,3),'-','Color',gcol(4),...
        'linewidth',2.5);
    
    groundplane(0.015);
    axis equal; grid on;
    axis([-0.05 0.05 -0.05 0.05 0 0.075]);
    title(['T = ',num2str(mdl.t(ii),3), '(s)']);
    view(30,30);
    drawnow;  
end


%% model-based controller
function tau = Controller(mdl)
t  = mdl.t;
Kp = diag([5, 0.001, 0.001]);
Kd = diag([1, 0.001, 0.001]);

Lambda = eye(3);
qd     = [0.01*sin(t) + 0.01; 30*sin(t); 30*cos(t)];
dqd    = [0.01*cos(t); 30*cos(t); -30*sin(t)];
ddqd   = [-0.01*sin(t); -30*sin(t); -30*cos(t)];

e    = mdl.q - qd;
de   = mdl.dq - dqd;
er   = de + Lambda*e;

dqr  = dqd  - Lambda*e;
ddqr = ddqd - Lambda*de;

f      = [0;0;0;0;0;-mdl.Pihat(8)*9.81];
deltaM = mdl.J.'*(adjointSE3inv(mdl.Phi,mdl.p)*f);

tau  = mdl.M*ddqr + mdl.C*dqr + mdl.G + mdl.K*mdl.q - Kp*e - Kd*er - deltaM;
end

function [dp,e] = UpdateLaw(mdl,alpha)
t  = mdl.t;
Y  = mdl.Y;
Ge = 10e3;
Gb = 6e-5;
Gm = 0.025;

Gamma  = alpha*diag([Ge,Ge,0,Gb,Gb,0,0,Gm]);
qd     = [0.01*sin(t) + 0.01; 30*sin(t); 30*cos(t)];
dqd    = [0.01*cos(t); 30*cos(t); -30*sin(t)];

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
    