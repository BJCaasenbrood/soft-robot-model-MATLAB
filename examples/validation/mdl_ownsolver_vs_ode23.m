addpath(genpath('src'));
close all; clear; clc;
%% set number of links
mdl = Model(5,'Tsim',15,'ODE','.','Conv',1e-4,'MaxItr',3);

%% settings
mdl = mdl.setElements(25);
mdl = mdl.setFrequency(50);
mdl = mdl.setLength(0.03);

%% simulate with hyper-elastic material
mdl     = mdl.set('ODE','.');
mdl.tau = @(mdl) Controller(mdl);
mdl     = mdl.simulate;
Q1      = mdl.q;

%% simulate with hyper-elastic material
mdl     = mdl.set('ODE','ode23t');
mdl.tau = @(mdl) Controller(mdl);
mdl     = mdl.simulate;
Q2      = mdl.q;

%% post process
l0  = mdl.get('l0');
l1   = mean(l0)*(1+Q1(:,1:3:3*mdl.Nlink));
kx1  = Q1(:,2:3:3*mdl.Nlink);
ky1  = Q1(:,3:3:3*mdl.Nlink);
l2   = mean(l0)*(1+Q2(:,1:3:3*mdl.Nlink));
kx2  = Q2(:,2:3:3*mdl.Nlink);
ky2  = Q2(:,3:3:3*mdl.Nlink);

f = figure(101); f.Name = 'Bishop parameters';
subplot(3,1,1); plot(mdl.t,l1 - l2,'linewidth',2); 
ylabel('$l(t)$','interpreter','latex','fontsize',20);
subplot(3,1,2); plot(mdl.t,kx1 - kx2,'linewidth',2); 
ylabel('$\kappa_x(t)$','interpreter','latex','fontsize',20);
subplot(3,1,3); plot(mdl.t,ky1 - ky2,'linewidth',2); 
ylabel('$\kappa_y(t)$','interpreter','latex','fontsize',20);

%% animate soft robot
f = figure(102);
f.Name = 'Own solver (blue) vs ODE23T (red)';

for ii = 1:fps(mdl.t,120):length(mdl.t)
    
    figure(102); cla;
    
    mdl.show(Q1(ii,:),'b');
    mdl.show(Q2(ii,:),'r');
    axis equal;
    
    axis(0.2*[-1 1 -1 1 -1 1]);
    view(0,0);
    drawnow;
    
end

function tau = Controller(mdl)
A = 0.02;
w = 1;
v = [0;sin(w*mdl.t^1.05);0];
tau = A*[v;zeros((mdl.Nlink-1)*3,1)];
end
    
% function tau = Controller(mdl)
% qd1 = [0;20;10];
% qd2 = [0;-20;0];
% qd3 = [0;0;30];
% qd4 = [0;-40*sin(mdl.t);0];
% qd = [qd1;qd2;qd3;qd4;qd2];
% KK = 1e-4*eye(3*5);
% tau = mdl.G + mdl.K*(mdl.q) - KK*(mdl.q - qd);
% end
%     