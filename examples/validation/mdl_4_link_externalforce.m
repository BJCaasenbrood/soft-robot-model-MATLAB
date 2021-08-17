addpath(genpath('src'));
close all; clear; clc;
%% set number of links
Ndis = 7;
mdl1 = Model(Ndis-4,'Tsim',5,'MaxItr',1);
mdl2 = Model(Ndis,'Tsim',5,'MaxItr',1);

%% settings
mdl1 = mdl1.setElements(75);
mdl1 = mdl1.setFrequency(300);
mdl1 = mdl1.setLength(0.064/mdl1.Nlink);
mdl1 = mdl1.setMass(0.05/mdl1.Nlink);
mdl2 = mdl2.setElements(75);
mdl2 = mdl2.setFrequency(300);
mdl2 = mdl2.setLength(0.064/mdl2.Nlink);
mdl2 = mdl2.setMass(0.05/mdl2.Nlink);

mdl1.q0(2) = 155/mdl1.Nlink;
mdl2.q0(2:3:end) = 155/mdl2.Nlink;
%% simulate with hyper-elastic material
mdl1.tau = @(mdl) Controller(mdl);
mdl1 = mdl1.simulate;
Q1  = mdl1.q;

%% simulate with hyper-elastic material
mdl2.tau = @(mdl) Controller(mdl);
mdl2 = mdl2.simulate;
Q2  = mdl2.q;

% %% post-processing data
% l0  = mdl.get('l0');
% l   = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
% kx  = mdl.q(:,2:3:3*mdl.Nlink);
% ky  = mdl.q(:,3:3:3*mdl.Nlink);
% dkx = mdl.dq(:,2:3:3*mdl.Nlink);
% dky = mdl.dq(:,2:3:3*mdl.Nlink);
% 
% f = figure(101); f.Name = 'Bishop parameters';
% subplot(3,1,1); plot(mdl.t,l,'linewidth',2); 
% ylabel('$l(t)$','interpreter','latex','fontsize',20);
% subplot(3,1,2); plot(mdl.t,kx,'linewidth',2); 
% ylabel('$\kappa_x(t)$','interpreter','latex','fontsize',20);
% subplot(3,1,3); plot(mdl.t,ky,'linewidth',2); 
% ylabel('$\kappa_y(t)$','interpreter','latex','fontsize',20);
% xlabel('time $t$ [s]','interpreter','latex','fontsize',20);

%% animate soft robot
f = figure(102);
f.Name = '';

for ii = 1:fps(mdl1.t,250):length(mdl1.t)
    
    figure(102); cla;
    
    mdl1.show(Q1(ii,:),'b');
    mdl2.show(Q2(ii,:),'r');
    axis equal;
    
    axis(0.2*[-1 1 -1 1 -1 1]);
    view(0,0);
    drawnow;
end

function tau = Controller(mdl)
P = mdl.Phi;
p = mdl.p;

Ad = adjointSE3inv(P,p);

J = mdl.J;
f = [0;0;0;0;0;-1];
tau = J.'*(Ad*f);
end
    
    