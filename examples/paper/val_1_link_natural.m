addpath(genpath('src'));
close all; clear; clc;

%%
load('oscill_data_b17.mat'); x1 = x; y1 = y;
load('oscill_data_b42.mat'); x2 = x; y2 = y;

%% set number of links
mdl = Model(1,'Tsim',.6,'Creep',true);

%% settings
mdl = mdl.setElements(30);
mdl = mdl.setFrequency(300);

mdl = mdl.setMass(0.017);
mdl = mdl.setRadius(0.013);
mdl = mdl.setDamping([0.01,3.02e-7]);

mdl = mdl.set('ke',[223.4, 174.0,-45.55]);
mdl = mdl.set('kb',[0.0131, 0.0123, -0.2129]);

mdl.tau = @(mdl) Controller(mdl);
%% simulate
mdl.lam0 = [0;0;1e-4];
mdl.q0 = [0,0,4.6];
mdl   = mdl.simulate;

%% post-processing data
l0 = mdl.get('l0');
l1  = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx1 = mdl.q(:,3:3:3*mdl.Nlink);
y1 = interp1(x1,y1,mdl.t);

%% simulate
mdl.lam0 = [0;0;1.7e-2];
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
plot(mdl.t,l1.*kx1*(180/pi),'-','Color',gcol(2),'linewidth',2.5); hold on;
plot(mdl.t,y1,'--','Color',gcol(2),'linewidth',2.5); 
plot(mdl.t,l2.*kx2*(180/pi),'-','Color',gcol(4),'linewidth',2.5); hold on;
plot(mdl.t,y2,'--','Color',gcol(4),'linewidth',2.5); 
ylabel('Bending angle $\beta$ (deg)','interpreter','latex','fontsize',19);
legend({'Simulation','Experiment','',''},'interpreter','latex','fontsize',18);
grid on; set(gca,'linewidth',1.5);

subplot(6,1,5:6);
plot(mdl.t,l1.*kx1*(180/pi) - y1,'-','Color',gcol(2),'linewidth',2.5); hold on;
plot(mdl.t,(l2.*kx2*(180/pi) - y2),'-','Color',gcol(4),'linewidth',2.5);
ylabel('Error (deg)','interpreter','latex','fontsize',19);
xlabel('time (s)','interpreter','latex','fontsize',19);
axis([0 0.6 -5 3]);
grid on; set(gca,'linewidth',1.5);

function tau = Controller(mdl)
    tau = mdl.q*0;
end
    
    