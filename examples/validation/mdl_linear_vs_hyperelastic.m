addpath(genpath('src'));
close all; clear; clc;
%% set number of links
mdl = Model(8);

%% settings
mdl = mdl.setElements(64);
mdl = mdl.setFrequency(60);
mdl = mdl.setLength(0.04);

%% simulate with hyper-elastic material
mdl     = mdl.set('ke',[223.435, 174.051, -45.5521]);
mdl     = mdl.set('kb',[0.42292, 0.39552, -0.21293]);
mdl.tau = @(mdl) Controller(mdl);
mdl     = mdl.simulate;
Q1      = mdl.q;

%% simulate with hyper-elastic material
mdl     = mdl.set('ke',[50,0,0]);   % optimized to match hyper-elastic - w = 2pi, A = 0.02;
mdl     = mdl.set('kb',[0.09,0,0]); % optimized to match hyper-elastic - w = 2pi, A = 0.02;
mdl.tau = @(mdl) Controller(mdl);
mdl     = mdl.simulate;
Q2      = mdl.q;

%% animate soft robot
f = figure(102);
f.Name = 'Hyper-elastic (blue) vs linear-elastic (red)';

for ii = 1:fps(mdl.t,30):length(mdl.t)
    
    figure(102); cla;
    
    groundplane(0.05);
    mdl.show(Q1(ii,:),col(1));
    mdl.show(Q2(ii,:),col(2));
    axis equal;
    
    axis(0.4*[-1 1 -1 1 -0.75 1.1]);
    view(0,0); drawnow;
    box on; grid on;
    set(gca,'linewidth',2.5);
    title('\color{blue} Hyper-elastic \color{black} vs. \color{red} Hookean',...
        'interpreter','tex','fontsize',18);
    background('w')
    
    if ii == 1, gif('hypervslinear.gif','frame',gcf,'nodither');
    else, gif;
    end
end

function tau = Controller(mdl)
A = 0.02;
w = 1;
v = [0;sin(w*mdl.t);0];
tau = A*[v;zeros((mdl.Nlink-1)*3,1)];
end
    
    