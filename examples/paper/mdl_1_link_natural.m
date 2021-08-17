addpath(genpath('src'));
close all; clear; clc;

%% set number of links
mdl = Model(1,'Tsim',0.5,'Creep',false);

%% settings
mdl = mdl.setElements(10);
mdl = mdl.setFrequency(1e3);

%% simulate hyper-elastic
mdl.q0 = [0.05,-15,15];
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
plot(t,l*1e3,'Color',greycolors(4),'linewidth',2.5); hold on;
ylabel('$l$ (mm)','interpreter','latex','fontsize',19);
axis([0 0.5 63 67]); grid on;
subplot(3,1,2); set(gca,'linewidth',1.5);
plot(t,kx,'Color',greycolors(4),'linewidth',2.5); hold on;
ylabel('$\kappa_x$ (m$^{-1}$)','interpreter','latex','fontsize',19); 
axis([0 0.5 -30 30]); grid on;
subplot(3,1,3); set(gca,'linewidth',1.5);
plot(t,ky,'Color',greycolors(4),'linewidth',2.5); hold on;
ylabel('$\kappa_y$ (m$^{-1}$)','interpreter','latex','fontsize',19); 
xlabel('time (s)','interpreter','latex','fontsize',19);
axis([0 0.5 -30 30]); grid on;

%% animate soft robot
for ii = 1:fps(mdl.t,500):length(mdl.t)
    figure(102); cla;
    P = mdl.show(mdl.q(ii,:));
    axis equal; axis(0.075*[-1 1 -1 1 0 1]); grid on;
    title(['T = ',num2str(mdl.t(ii),3), '(s)']);
    view(30,30);
    drawnow;  
end

    
    