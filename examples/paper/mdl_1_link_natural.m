addpath(genpath('src'));
close all; clear; clc;

%% set number of links
mdl = Model(1,'Tsim',0.5,'Creep',true,'Linewidth',3);

%% settings
mdl = mdl.setElements(30);
mdl = mdl.setFrequency(1e3);
mdl = mdl.setMass(0.0105);
mdl = mdl.setRadius(0.0123);
mdl = mdl.setDamping([0.01,2.3e-7]);

mdl = mdl.set('ke',[223.4, 174.0,-45.55]);
mdl = mdl.set('kb',[0.0130, 0.0124, -0.2129]);

%% simulate hyper-elastic
mdl.q0  = [0,-15,15];
mdl.dq0 = [0,2500,0];
mdl     = mdl.simulate;

%% post-processing data
t  = mdl.t;
l0 = mdl.get('l0');
l  = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
ee = mdl.q(:,1:3:3*mdl.Nlink);
kx = mdl.q(:,2:3:3*mdl.Nlink);
ky = mdl.q(:,3:3:3*mdl.Nlink);
dkx = mdl.dq(:,2:3:3*mdl.Nlink);
dky = mdl.dq(:,2:3:3*mdl.Nlink);

f = figure(101); f.Name = 'Bishop parameters';
subplot(3,1,1); set(gca,'linewidth',1.5);
plot(t,ee,'Color',col(4),'linewidth',2.5); hold on;
ylabel('$\varepsilon$ (-)','interpreter','latex','fontsize',18);
%axis([0 mdl.t(end) 63.75 64.75]); 
grid on;
subplot(3,1,2); set(gca,'linewidth',1.5);
plot(t,kx,'Color',col(4),'linewidth',2.5); hold on;
ylabel('$\kappa_x$ (m$^{-1}$)','interpreter','latex','fontsize',18); 
axis([0 mdl.t(end) -30 30]); grid on;
subplot(3,1,3); set(gca,'linewidth',1.5);
plot(t,ky,'Color',col(4),'linewidth',2.5); hold on;
ylabel('$\kappa_y$ (m$^{-1}$)','interpreter','latex','fontsize',18); 
xlabel('time (s)','interpreter','latex','fontsize',18);
axis([0 mdl.t(end) -30 30]); grid on;

error('');

%% recover trajectory end effector
P = zeros(length(mdl.t),3);
for ii = 1:length(mdl.t)
    [p0, ~] = mdl.computeEndEffector(mdl.q(ii,:),mdl.dq(ii,:));
    P(ii,:) = p0.';
end

%% animate soft robot
for ii = 1:fps(mdl.t,500):length(mdl.t)
    figure(102); cla;
    
    mdl.show(mdl.q(ii,:),col(3));
    
    plot3(P(1:ii,1),P(1:ii,2),P(1:ii,3),'-','Color',col(4),...
        'linewidth',2.5);
    
    groundplane(0.015);
    axis equal; grid on;
    axis([-0.04 0.04 -0.04 0.04 0 0.075]);
    title(['T = ',num2str(mdl.t(ii),3), '(s)']);
    view(30,30);
    drawnow;  
end

    
    