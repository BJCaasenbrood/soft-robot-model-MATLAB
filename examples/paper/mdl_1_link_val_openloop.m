addpath(genpath('src'));
close all; clear; clc;

%% experimental load data
load('openloop_data_p123.mat');

nf = 50;

U_ = [smooth(U_(:,1),nf),...
      smooth(U_(:,2),nf),...
      smooth(U_(:,3),nf)];

kx_ = smooth(kx_,50);
ky_ = smooth(ky_,50);

%% set number of links
mdl = Model(1,'Tsim',40,'Creep',true);

%% settings
mdl = mdl.setElements(10);
mdl = mdl.setFrequency(30);
mdl = mdl.setLength(0.064);

%% interpolate data signals
U1_ = interp1(stepspace(0,40,1/250),U_(:,1),mdl.t);
U2_ = interp1(stepspace(0,40,1/250),U_(:,2),mdl.t);
U3_ = interp1(stepspace(0,40,1/250),U_(:,3),mdl.t);
kx_ = interp1(stepspace(0,40,1/250),kx_,mdl.t);
ky_ = interp1(stepspace(0,40,1/250),ky_,mdl.t);

U_ = [U1_,U2_,U3_];

%% set controller
mdl.tau = @(mdl) Controller(mdl,U_);

%% simulate hyper-elastic
mdl = mdl.simulate;
kx1 = mdl.q(:,2:3:3*mdl.Nlink);
ky1 = mdl.q(:,3:3:3*mdl.Nlink);

%% simulate
mdl = mdl.set('ke',[50.02,0,0],'kb',[0.027,0,0],'kp',0);
mdl.tau = @(mdl) Controller(mdl,U_);
mdl = mdl.simulate;
kx2 = mdl.q(:,2:3:3*mdl.Nlink);
ky2 = mdl.q(:,3:3:3*mdl.Nlink);

%% plotting
figure(101); clf;
subplot(3,4,1:4); 

plot(mdl.t,U_(:,1)/1e3,'-',...
    'Color',greycolors(2),'linewidth',2.5);  hold on; 
plot(mdl.t,U_(:,2)/1e3,'-',...
    'Color',greycolors(3),'linewidth',2.5);
plot(mdl.t,U_(:,3)/1e3,'-',...
    'Color',greycolors(4),'linewidth',2.5);
legend('$u_1$','$u_2$','$u_3$','Simulation','Location','NorthWest',...
    'Orientation','horizontal','interpreter','latex','fontsize',19);
set(gca,'linewidth',1.5);

axis([0 40 5 40]);
xlabel('time (s)','interpreter','latex','fontsize',19);
ylabel('$u$ (kPa)','interpreter','latex','fontsize',19);
grid on; set(gca,'linewidth',1.5);

subplot(3,4,[5,6,9,10]); 
plot(kx_-0.36,ky_ -0.1,':','Color',greycolors(2),'linewidth',2.5); 

hold on;
V1 = rotx(-0.5)*[0*kx1.';ky1.';-kx1.'];
plot(V1(2,:),V1(3,:),'Color',greycolors(4),'linewidth',2.5); 
axis([-9.5 9.5 -8 12]); axis square; grid on; set(gca,'linewidth',1.5);
xlabel('$\kappa_x$ (m$^{-1}$)','interpreter','latex','fontsize',19);
ylabel('$\kappa_y$ (m$^{-1}$)','interpreter','latex','fontsize',19);
legend('Experiment','FEM-driven model','interpreter','latex','fontsize',17);
set(gca,'linewidth',1.5);

subplot(3,4,[7,8,11,12]); 
plot(kx_-0.36,ky_ -0.1,':','Color',greycolors(2),'linewidth',2.5); 

hold on;
V2 = rotx(-0.5)*[0*kx2.';ky2.';-kx2.'];
plot(V2(2,:),V2(3,:),'Color',greycolors(4),'linewidth',2.5); 
axis([-9.5 9.5 -8 12]); axis square; grid on; set(gca,'linewidth',1.5);
xlabel('$\kappa_x$ (m$^{-1}$)','interpreter','latex','fontsize',19);
legend('Experiment','Hookean model','interpreter','latex','fontsize',17);

%% animate soft robot
% for ii = 1:fps(mdl.t,12):length(mdl.t)
%     figure(102); cla;
%     mdl.show(mdl.q(ii,:));
%     axis equal; axis(0.075*[-1 1 -1 1 0 1]); grid on;
%     title(['T = ',num2str(mdl.t(ii),3), '(s)']);
%     view(30,30);
%     drawnow;  
% end

function tau = Controller(mdl,Y)
r0 = 10;
r1 = 1;

alp = [0, 2*pi/3, 4*pi/3];
H   = 5.7e-7*[r0,r0,r0;...
              r1*sin(alp(1)), r1*sin(alp(2)), r1*sin(alp(3));
             -r1*cos(alp(1)),-r1*cos(alp(2)),-r1*cos(alp(3))];

% P   = @(x,a) 25e3*(0.5*sin(x + a) + 0.5).*min(0.05*x,1) + 10e3;         
% tau = H*[P(t,0);P(t,2*pi/3);P(t,4*pi/3)];

ii = max(round((mdl.t)/mdl.Tstep),1);
tau =  H*[Y(ii,1);Y(ii,2);Y(ii,3)];
end
    
    