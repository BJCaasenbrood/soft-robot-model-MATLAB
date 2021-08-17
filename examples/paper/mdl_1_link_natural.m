addpath(genpath('src'));
close all; clear; clc;

%% set number of links
mdl = Model(1,'Tsim',0.5,'Creep',true);

%% settings
mdl = mdl.setElements(30);
mdl = mdl.setFrequency(1e3);

%% simulate hyper-elastic
mdl.q0 = [0.0,0,15];
mdl.dq0 = [0,2500,0];
mdl = mdl.simulate;

%% post-processing data
t  = mdl.t;
l0 = mdl.get('l0');
l  = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx = mdl.q(:,2:3:3*mdl.Nlink);
ky = mdl.q(:,3:3:3*mdl.Nlink);
dkx = mdl.dq(:,2:3:3*mdl.Nlink);
dky = mdl.dq(:,2:3:3*mdl.Nlink);

f = figure(101); f.Name = 'Bishop parameters';
subplot(3,1,1); set(gca,'linewidth',1.5);
plot(t,l*1e3,'Color',col(1),'linewidth',1.5); hold on;
ylabel('$l$ (mm)','interpreter','latex','fontsize',12);
axis([0 0.5 63.75 64.75]); grid on;
subplot(3,1,2); set(gca,'linewidth',1.5);
plot(t,kx,'Color',col(1),'linewidth',1.5); hold on;
ylabel('$\kappa_x$ (m$^{-1}$)','interpreter','latex','fontsize',12); 
axis([0 0.5 -30 30]); grid on;
subplot(3,1,3); set(gca,'linewidth',1.5);
plot(t,ky,'Color',col(1),'linewidth',1.5); hold on;
ylabel('$\kappa_y$ (m$^{-1}$)','interpreter','latex','fontsize',12); 
xlabel('time (s)','interpreter','latex','fontsize',12);
axis([0 0.5 -30 30]); grid on;

%% recover trajectory end effector
P = zeros(length(mdl.t),3);
for ii = 1:length(mdl.t)
    [p0, ~] = mdl.computeEndEffector(mdl.q(ii,:),mdl.dq(ii,:));
    P(ii,:) = p0.';
end

%% animate soft robot
for ii = 1:fps(mdl.t,500):length(mdl.t)
    figure(102); cla;
    
    mdl.show(mdl.q(ii,:),col(1));
    
    plot3(P(1:ii,1),P(1:ii,2),P(1:ii,3),'-','Color',col(1),...
        'linewidth',1.5);
    
    groundplane(0.015);
    axis equal; axis(0.075*[-0.75 0.75 -0.75 0.75 0 1]); grid on;
    title(['T = ',num2str(mdl.t(ii),3), '(s)']);
    view(30,30);
    drawnow;  
end

    
    