addpath(genpath('src'));
close all; clear; clc;

%%
load('oscill_data_b17.mat'); x1 = x; y1 = y;
load('oscill_data_b42.mat'); x2 = x; y2 = y;

%% set number of links
mdl = Model(1,'Tsim',0.6,'Creep',true);

%% settings
mdl = mdl.setElements(30);
mdl = mdl.setFrequency(300);

mdl.tau = @(mdl) Controller(mdl);
%% simulate
mdl.lam0 = [0;0;0.3];
mdl.q0   = [0,0,4.6];
mdl      = mdl.simulate;

%% post-processing data
l0 = mdl.get('l0');
l1  = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx1 = mdl.q(:,3:3:3*mdl.Nlink);
y1 = interp1(x1,y1,mdl.t);

%% simulate
mdl.lam0 = [0;0;1.3];
mdl.q0   = [0,0,11.6];
mdl.tau  = @(mdl) Controller(mdl);
mdl      = mdl.simulate;

%% post-processing data
l0 = mdl.get('l0');
l2 = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx2 = mdl.q(:,3:3:3*mdl.Nlink);
y2 = interp1(x2,y2,mdl.t);

figure(101);
subplot(6,1,1:4);
plot(mdl.t,l1.*kx1*(180/pi),'-','Color',greycolors(2),'linewidth',2.5); hold on;
plot(mdl.t,y1,'--','Color',greycolors(2),'linewidth',2.5); 
plot(mdl.t,l2.*kx2*(180/pi),'-','Color',greycolors(4),'linewidth',2.5); hold on;
plot(mdl.t,y2,'--','Color',greycolors(4),'linewidth',2.5); 
ylabel('Bending angle $\beta$ (deg)','interpreter','latex','fontsize',19);
legend({'Simulation','Experiment','',''},'interpreter','latex','fontsize',18);
grid on; set(gca,'linewidth',1.5);

subplot(6,1,5:6);
plot(mdl.t,l1.*kx1*(180/pi) - y1,'-','Color',greycolors(2),'linewidth',2.5); hold on;
plot(mdl.t,(l2.*kx2*(180/pi) - y2),'-','Color',greycolors(4),'linewidth',2.5);
ylabel('Error (deg)','interpreter','latex','fontsize',19);
xlabel('time (s)','interpreter','latex','fontsize',19);
axis([0 0.6 -2.2 1.5]);
grid on; set(gca,'linewidth',1.5);

function tau = Controller(mdl)
    tau = mdl.q*0;
end
    
    