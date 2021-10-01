addpath(genpath('src'));
close all; clear; clc;
%% set number of links
mdl = Model(4,'Tsim',15);

%% settings
mdl = mdl.setElements(40);
mdl = mdl.setFrequency(30);
mdl = mdl.setLength(0.04);
mdl = mdl.setMass(0.02);
mdl = mdl.setLoad(0.05);

%% simulate with hyper-elastic material
mdl.q0(2:3:end) = 1;
mdl = mdl.simulate;

%% post-processing data
l0  = mdl.get('l0');
l   = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx  = mdl.q(:,2:3:3*mdl.Nlink);
ky  = mdl.q(:,3:3:3*mdl.Nlink);
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

%% animate soft robot
f = figure(102);
f.Name = '';

for ii = 1:fps(mdl.t,15):length(mdl.t)
    
    figure(102); cla;
    
    mdl.show(mdl.q(ii,:),'b');
    axis equal;
    
    axis(0.2*[-1 1 -1 1 -1 1]);
    view(0,0);
    drawnow;
end
