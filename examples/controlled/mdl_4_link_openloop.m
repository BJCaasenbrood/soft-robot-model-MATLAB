addpath(genpath('src'));
close all; clear; clc;
%% set number of links
mdl = Model(4);

%% settings
mdl = mdl.set('Phi0',rotx(pi),'Tsim',15,'MaxItr',10,'Creep',false);
mdl = mdl.setElements(40);
mdl = mdl.setFrequency(50);
mdl = mdl.setLength(0.045);

mdl = mdl.setControl( @(mdl) Controller(mdl) );

%% simulate with initial conditions
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

%% plot phase portaits
f = figure(102); f.Name = 'Phase portrait';
subplot(2,2,1); plot(kx(:,1),dkx(:,1),'linewidth',2); 
subplot(2,2,2); plot(kx(:,2),dkx(:,2),'linewidth',2); 
subplot(2,2,3); plot(kx(:,3),dkx(:,3),'linewidth',2); 
subplot(2,2,4); plot(kx(:,4),dkx(:,4),'linewidth',2); 

%% play animation soft robot
f = figure(103);
Q = mdl.q;

for ii = 1:fps(mdl.t,60):length(mdl.t)
    
    figure(103); cla;
    P = mdl.show(Q(ii,:));
    axis equal; axis(0.35*[-1 1 -1 1 -1 0.1]);
    f.Name = [' Time =',num2str(mdl.t(ii),3)];
    view(0,0);
    drawnow();
    
    
end

%% model-based controller
function tau = Controller(mdl)
v1  = [0;sin(6.5*mdl.t);0];
tau = 0.02*[zeros((mdl.Nlink-1)*3,1);v1];
end
    
    