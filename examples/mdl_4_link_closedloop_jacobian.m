addpath(genpath('src'));
close all; clear; clc;
%% set number of links
mdl = Model(4);

%% settings
mdl = mdl.set('Phi0',rotx(pi));
mdl = mdl.setElements(60);
mdl = mdl.setFrequency(60);
mdl = mdl.setLength(0.065);

mdl = mdl.setControl( @(mdl) Controller(mdl) );

%% simulate with initial conditions
mdl = mdl.simulate;

%% post-processing data
l0 = mdl.get('l0');
l  = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx = mdl.q(:,2:3:3*mdl.Nlink);
ky = mdl.q(:,3:3:3*mdl.Nlink);
dkx = mdl.dq(:,2:3:3*mdl.Nlink);
dky = mdl.dq(:,2:3:3*mdl.Nlink);

f = figure(101); f.Name = 'Bishop parameters';
subplot(3,1,1); plot(mdl.t,l,'linewidth',2); 
ylabel('$l(t)$','interpreter','latex','fontsize',20);
subplot(3,1,2); plot(mdl.t,kx,'linewidth',2); 
ylabel('$\kappa_x(t)$','interpreter','latex','fontsize',20);
subplot(3,1,3); plot(mdl.t,ky,'linewidth',2); 
ylabel('$\kappa_y(t)$','interpreter','latex','fontsize',20);
xlabel('time $t$ [s]','interpreter','latex','fontsize',20);

%% play animation soft robot
f = figure(103);
Q = mdl.q;
Qd = [0;20;10;0;-20;0;0;0;30;0;-40;0].' + zeros(length(mdl.t),12);

for ii = 1:fps(mdl.t,12):length(mdl.t)
    figure(103); cla;
    mdl.show(Q(ii,:),col(1));
    mdl.show(Qd(ii,:),col(2));
    groundplane(0.02);
    axis equal; axis(0.2*[-0.75 0.75 -0.75 0.75 -1.5 0.1]);
    f.Name = [' Time =',num2str(mdl.t(ii),3)];
        
    title('\color{blue} Soft manipulator (N=4) \color{black} vs. \color{red} Desired',...
        'interpreter','tex','fontsize',12);
    
    drawnow(); grid on; box on; 
    view(30,30); set(gcf,'color','w');
    set(gca,'linewidth',1.5);
    
%     if ii == 1, gif('mdl_4_closedloop_jac.gif','frame',gcf,'nodither');
%     else, gif;
%     end
end

%% model-based controller
function tau = Controller(mdl)
p  = mdl.p;
J  = mdl.J(4:end,:);
xd = [0.1;-0.0;-0.1];

lamb = 0.001;
PJD = J.'*inv(J*J.' - lamb*eye(3));

e = (p - xd);

Kp = 0.01;
Kd = 3e-4*eye(12);
Ke = kron(eye(mdl.Nlink),diag([0.001,1,1]));
tau = mdl.G - Ke*mdl.K*mdl.q + Kp*PJD*e - Kd*(mdl.dq);
end
    
    