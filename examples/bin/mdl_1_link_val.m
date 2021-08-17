addpath(genpath('src'));
close all; clear; clc;

%%
load('oscill_data_b17.mat'); x1 = x; y1 = y;
load('oscill_data_b42.mat'); x2 = x; y2 = y;

%% set number of links
mdl = Model(1,'Tsim',0.6);

%% settings
mdl = mdl.setElements(60);
mdl = mdl.setFrequency(500);

mdl.tau = @(mdl) Controller(mdl);
%% simulate
mdl.q0  = [0,4.75,0];
mdl = mdl.simulate;

%% post-processing data
l0 = mdl.get('l0');
l1  = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx1 = mdl.q(:,2:3:3*mdl.Nlink);

%% simulate
mdl.tau = @(mdl) Controller(mdl);
mdl.q0  = [0,12,0];
mdl = mdl.simulate;

%% post-processing data
l0 = mdl.get('l0');
l2 = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx2 = mdl.q(:,2:3:3*mdl.Nlink);

figure(101);
plot(mdl.t,l1.*kx1*(180/pi),'b','linewidth',2); hold on;
plot(x1,y1,'b--','linewidth',2); 
plot(mdl.t,l2.*kx2*(180/pi),'r','linewidth',2); hold on;
plot(x2,y2,'r--','linewidth',2); 
ylabel('$\beta(t)$','interpreter','latex');
xlabel('t [s]','interpreter','latex');

function tau = Controller(mdl)
    tau = mdl.q*0;
end
    
    