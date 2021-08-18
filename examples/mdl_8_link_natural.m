addpath(genpath('src'));
close all; clear; clc;
%% set number of links
mdl = Model(8);

%% settings
mdl = mdl.set('Tsim',10);
mdl = mdl.setElements(40);
mdl = mdl.setFrequency(30);
mdl = mdl.setLength(0.03);

%% simulate with non-zero initial conditions
mdl.q0(2:3:end) = 5;
mdl = mdl.simulate;

%% play animation soft robot
for ii = 1:fps(mdl.t,15):length(mdl.t)
    figure(102); cla;
    
    groundplane(0.03);
    mdl.show(mdl.q(ii,:),col(1));
    
    axis equal; axis(0.25*[-0.4 1.25 -1 1 -1 1]);
    view(0,0); box on; grid on;
    set(gca,'linewidth',1.5);
end

    