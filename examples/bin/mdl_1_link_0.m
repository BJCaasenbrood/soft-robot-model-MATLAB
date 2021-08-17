addpath(genpath('src'));
close all; clear; clc;
%% set number of links
mdl = Model(1,'Tsim',1.5);

%% settings
mdl = mdl.setElements(60);
mdl = mdl.setFrequency(500);

mdl.tau = @(mdl) Controller(mdl);
%% simulate
mdl.q0  = [0,-0,15];
mdl.dq0 = [0,2500,0];
mdl = mdl.simulate;

%% post-processing data
l0 = mdl.get('l0');
l  = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx = mdl.q(:,2:3:3*mdl.Nlink);
ky = mdl.q(:,3:3:3*mdl.Nlink);

figure(101);
subplot(3,1,1); plot(mdl.t,l,'linewidth',2); 
ylabel('$l(t)$','interpreter','latex');
subplot(3,1,2); plot(mdl.t,kx,'linewidth',2); 
ylabel('$\kappa_x(t)$','interpreter','latex');
subplot(3,1,3); plot(mdl.t,ky,'linewidth',2); 
ylabel('$\kappa_y(t)$','interpreter','latex');
xlabel('t [s]','interpreter','latex');

%% animate soft robot
figure(102);

for ii = 1:fps(mdl.t,120):length(mdl.t)
    
    figure(102); cla;
    
    mdl.show(mdl.q(ii,:));
    axis equal; axis(0.075*[-1 1 -1 1 0 1]); grid on;
    title(['T = ',num2str(mdl.t(ii),3), '(s)']);
    view(30,30);
    drawnow;
end

function tau = Controller(mdl)
tau = mdl.q*0;
end
    
    