addpath(genpath('src'));
close all; clear; clc;
%% set number of links
mdl = Model(8);

%% settings
mdl = mdl.set('Tsim',25);
mdl = mdl.setElements(50);
mdl = mdl.setFrequency(50);
mdl = mdl.setLength(0.04);

mdl.tau = @(mdl) Controller(mdl);
%% simulate with initial conditions
mdl.q0(2:3:end) = (rand(8,1));
mdl = mdl.simulate;

%% post-processing data
l0 = mdl.get('l0');
l  = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx = mdl.q(:,2:3:3*mdl.Nlink);
ky = mdl.q(:,3:3:3*mdl.Nlink);

figure(101);
subplot(3,1,1); plot(mdl.t,l,'linewidth',2);
subplot(3,1,2); plot(mdl.t,kx,'linewidth',2);
subplot(3,1,3); plot(mdl.t,ky,'linewidth',2);

%% animate soft robot
figure(102);
Q = mdl.q;

for ii = 1:fps(mdl.t,12):length(mdl.t)
    
    figure(102); cla;
    
    mdl.show(Q(ii,:));
    axis equal;
    axis(0.75*[-1 1 -1 1 -1 1]);
    drawnow();
    
end

function tau = Controller(mdl)
qd = mdl.q0;
qd(11) = 15;

tau = mdl.G*0;
end
    
    