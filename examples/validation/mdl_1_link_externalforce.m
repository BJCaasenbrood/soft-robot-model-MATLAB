addpath(genpath('src'));
close all; clear; clc;
%% set number of links
Ndis = 2;
mdl = Model(Ndis,'Tsim',5,'MaxItr',1);

%% settings
mdl = mdl.setElements(75);
mdl = mdl.setFrequency(70);
mdl = mdl.setLength(0.064/mdl.Nlink);
mdl = mdl.setMass(0.05/mdl.Nlink);

mdl.q0(2:3:end) = 55/mdl.Nlink;

%% simulate with hyper-elastic material
mdl.tau = @(mdl) Controller(mdl);
mdl = mdl.simulate;
Q  = mdl.q;

%% post-processing data
l0  = mdl.get('l0');
l   = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx  = mdl.q(:,2:3:3*mdl.Nlink);
ky  = mdl.q(:,3:3:3*mdl.Nlink);
dkx = mdl.dq(:,2:3:3*mdl.Nlink);
dky = mdl.dq(:,2:3:3*mdl.Nlink);
% 
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

for ii = 1:fps(mdl.t,250):length(mdl.t)
    
    figure(102); cla;
    
    mdl.show(Q(ii,:),'b');
    axis equal;
    
    axis(0.2*[-1 1 -1 1 -1 1]);
    view(0,0);
    drawnow;
end

function tau = Controller(mdl)
P = mdl.Phi;
p = mdl.p;

Ad  = adjointSE3(P,p);
Adi = adjointSE3inv(P,p);

J = mdl.J;
f = [0;0;0;0;0;-5];
tau = (J).'*(Adi*f);
end
    
    