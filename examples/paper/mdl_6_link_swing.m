%addpath(genpath('src'));
%close all; clear; clc;
%% set number of links
mdl = Model(6);

%% settings
mdl = mdl.set('Tsim',10);
mdl = mdl.setElements(s);
mdl = mdl.setFrequency(h);
mdl = mdl.setLength(0.025);

%% set controller
mdl.tau = @(mdl) Controller(mdl);

%% simulate with initial conditions
mdl.q0(2:3:end) = 1;
tic
mdl = mdl.simulate;
T = toc;

P = zeros(length(mdl.t),4);
for ii = 1:length(mdl.t)
   P(ii,:) = [mdl.t(ii), mdl.computeEndEffector(mdl.q(ii,:),0).'];
   %ii
end
% 
% clearvars -except mdl T P
% error('0');

% %% post-processing data
% t  = mdl.t;
% l0 = mdl.get('l0');
% l  = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
% ee = mdl.q(:,1:3:3*mdl.Nlink);
% kx = mdl.q(:,2:3:3*mdl.Nlink);
% ky = mdl.q(:,3:3:3*mdl.Nlink);
% dkx = mdl.dq(:,2:3:3*mdl.Nlink);
% dky = mdl.dq(:,2:3:3*mdl.Nlink);
% 
% f = figure(101); f.Name = 'Bishop parameters';
% subplot(2,1,1); set(gca,'linewidth',1.5);
% plot(t,ee,'linewidth',2.5); hold on;
% ylabel('$\varepsilon$ (-)','interpreter','latex','fontsize',18);
% colororder(repmat(linspace(0.7,0.1,6).',1,3))
% %axis([0 mdl.t(end) 63.75 64.75]); 
% grid on;
% subplot(2,1,2); set(gca,'linewidth',1.5);
% plot(t,kx,'linewidth',2.5); hold on;
% ylabel('$\kappa$ (m$^{-1}$)','interpreter','latex','fontsize',18); 
% axis([0 mdl.t(end) -110 110]); grid on;
% subplot(2,1,2); set(gca,'linewidth',1.5);
% colororder(repmat(linspace(0.7,0.1,6).',1,3))
% xlabel('time (s)','interpreter','latex','fontsize',18);
% error('');
% %% play animation soft robot
% f = figure(103); 
% Q = mdl.q;
% 
% %% recover trajectory end effector
% P = zeros(length(mdl.t),3);
% for ii = 1:length(mdl.t)
%     [p0, ~] = mdl.computeEndEffector(mdl.q(ii,:),mdl.dq(ii,:));
%     P(ii,:) = p0.';
% end
% 
% %% animate soft robot
% % for ii = 1:fps(mdl.t,20):length(mdl.t)
% %     figure(103); cla;
% %     
% %     mdl.show(mdl.q(ii,:),gcol(3));
% %     
% %     %plot3(P(1:ii,1),P(1:ii,2),P(1:ii,3),'-','Color',gcol(4),...
% %     %    'linewidth',2.5);
% %     
% %     C = repmat(linspace(0.1,0.7,ii).',1,3);
% %     
% %     clinep(P(1:ii,1),P(1:ii,2),P(1:ii,3),C,2.5);
% %     
% %     groundplane(0.02);
% %     axis equal;
% %     axis(0.15*[-1 1 -1 1 -0.75 1]);
% %     view(0,0); drawnow;
% % end
% 
% figure(103); clf;
% % 
% clinep(P(1:end,1),P(1:end,2)+0.01,P(1:end,3),linspace(0,1,length(1:1200)),3); hold on;
% mdl.show(mdl.q(1,:),gcol(4));
% %mdl.show(mdl.q(50,:),gcol(4));
% mdl.show(mdl.q(400,:),gcol(4));
% mdl.show(mdl.q(519,:),gcol(4));
% mdl.show(mdl.q(535,:),gcol(4));
% mdl.show(mdl.q(656,:),gcol(4));
% %mdl.show(mdl.q(430,:),gcol(4));
% %mdl.show(mdl.q(550,:),gcol(4));
% 
% view(0,0);
% groundplane(0.02);
% axis equal; grid on;
% axis(0.175*[-1 1 -1 1 -0.75 1]);
% set(gca,'linewidth',1.5);
% drawnow;
% colormap(flipud(repmat(linspace(0.4,0.9,length(P))',1,3)));
% 
% 
% %%
% 
% for ii = 1:fps(mdl.t,25):length(mdl.t)
%     figure(103); cla;
%     mdl.show(Q(ii,:),gcol(4));
%     groundplane(0.02);
%     axis equal;
%     axis(0.15*[-1 1 -1 1 -0.75 1]);
%     view(0,0); drawnow;
%     f.Name = [' Time =',num2str(mdl.t(ii),3)];
%     drawnow();
%     grid on;
%     %axis equal
% end

function tau = Controller(mdl)
A = 0.02;
w = pi;
v = [0;sin(w*mdl.t);0];
tau = A*[v;zeros((mdl.Nlink-1)*3,1)];
end
    

    