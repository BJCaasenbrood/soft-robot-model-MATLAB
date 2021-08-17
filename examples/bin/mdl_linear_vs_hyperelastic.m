addpath(genpath('src'));
close all; clear; clc;
%% set number of links
mdl = Model(6);

%% settings
mdl = mdl.setElements(50);
mdl = mdl.setFrequency(120);
mdl = mdl.setLength(0.03);

%% simulate with hyper-elastic material
mdl     = mdl.set('ke',[223.435, 174.051, -45.5521]);
mdl     = mdl.set('kb',[0.42292, 0.39552, -0.21293]);
mdl.tau = @(mdl) Controller(mdl);
mdl     = mdl.simulate;
Q1      = mdl.q;

%% simulate with hyper-elastic material
mdl     = mdl.set('ke',[50,0,0]);   % optimized to match hyper-elastic - w = 2pi, A = 0.02;
mdl     = mdl.set('kb',[0.07,0,0]); % optimized to match hyper-elastic - w = 2pi, A = 0.02;
mdl.tau = @(mdl) Controller(mdl);
mdl     = mdl.simulate;
Q2      = mdl.q;

%% animate soft robot
f = figure(102);
f.Name = 'Hyper-elastic (blue) vs linear-elastic (red)';

for ii = 1:fps(mdl.t,120):length(mdl.t)
    
    figure(102); cla;
    
    mdl.show(Q1(ii,:),'b');
    mdl.show(Q2(ii,:),'r');
    axis equal;
    
    axis(0.2*[-1 1 -1 1 -1 1]);
    view(0,0);
    drawnow;
    
end

function tau = Controller(mdl)
A = 0.02;
w = 2*pi;
v = [0;sin(w*mdl.t);0];
tau = A*[v;zeros((mdl.Nlink-1)*3,1)];
end
    
    