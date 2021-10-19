%close all; clear; clc;

%% set number of links
mdl = Model(1,'Tsim',15);

%% settings
mdl = mdl.setElements(10);
mdl = mdl.setFrequency(15);

mdl = mdl.setMass(0.017);
mdl = mdl.setRadius(0.013);
mdl = mdl.setDamping([0.01,3.02e-7]);

mdl = mdl.set('ke',[223.4, 174.0,-45.55]);
mdl = mdl.set('kb',[0.0130, 0.0124, -0.2129]);

mdl.q0(2) = 1;
%% set loads
M = [0.05,0.1,0.15];

%% simulate hyper-elastic
for ii = 1:numel(M)
    mdl.tau = @(x) 0;
    mdl = mdl.setLoad(M(ii));
    mdl = mdl.simulate;
    
    mdl.show(mdl.q(end,:),gcol(ii+1)); hold on;
    axis equal;
    view(0,0)
end

    