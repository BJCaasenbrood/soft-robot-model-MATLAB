close all; clear; clc;
%% experimental load data
load('openloop_data_p123.mat');

%% set number of links
mdl = Model(1,'Tsim',40,'Creep',true);

%% settings
mdl = mdl.setElements(15);
mdl = mdl.setFrequency(25);

mdl = mdl.setMass(0.017);
mdl = mdl.setRadius(0.013);
mdl = mdl.setDamping([0.01,3.02e-7]);

mdl = mdl.set('ke',[223.4, 174.0,-45.55]);
mdl = mdl.set('kb',[0.0131, 0.0123, -0.2129]);

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
mdl = mdl.set('ke',[50.02,0,0],'kb',[8.25e-4,0,0],'kp',0);
mdl.tau = @(mdl) Controller(mdl,U_);

mdl = mdl.simulate;
kx2 = mdl.q(:,2:3:3*mdl.Nlink);
ky2 = mdl.q(:,3:3:3*mdl.Nlink);

%% plotting
figure(101); clf;
subplot(3,4,1:4); 

plot(mdl.t,U_(:,1)/1e3,'-',...
    'Color',gcol(2),'linewidth',2);  hold on; 
plot(mdl.t,U_(:,2)/1e3,'-',...
    'Color',gcol(3),'linewidth',2);
plot(mdl.t,U_(:,3)/1e3,'-',...
    'Color',gcol(4),'linewidth',2);
legend({'$u_1$','$u_2$','$u_3$'},'Location','NorthWest',...
    'Orientation','horizontal','interpreter','latex','fontsize',10);
set(gca,'linewidth',1);

axis([0 40 5 40]);
xlabel('time (s)','interpreter','latex','fontsize',12);
ylabel('$u$ (kPa)','interpreter','latex','fontsize',12);
grid on; set(gca,'linewidth',1.5);

subplot(3,4,[5,6,9,10]); 
plot(kx_-0.36,ky_ -0.1,':','Color',gcol(2),'linewidth',2); 

hold on;
V1 = rotx(-0.5)*[0*kx1.';ky1.';-kx1.'];
plot(V1(2,:),V1(3,:),'Color',gcol(4),'linewidth',2); 
axis([-9.5 9.5 -8 12]); axis square; grid on; set(gca,'linewidth',2);
xlabel('$\kappa_x$ (m$^{-1}$)','interpreter','latex','fontsize',12);
ylabel('$\kappa_y$ (m$^{-1}$)','interpreter','latex','fontsize',12);
legend({'Experiment','Hyper-elastic'},'interpreter','latex','fontsize',17);
set(gca,'linewidth',1);

subplot(3,4,[7,8,11,12]); 
plot(kx_-0.36,ky_ -0.1,':','Color',gcol(2),'linewidth',2); 

hold on;
V2 = rotx(-0.5)*[0*kx2.';ky2.';-kx2.'];
plot(V2(2,:),V2(3,:),'Color',gcol(4),'linewidth',2); 
axis([-9.5 9.5 -8 12]); axis square; grid on; set(gca,'linewidth',1);
xlabel('$\kappa_x$ (m$^{-1}$)','interpreter','latex','fontsize',12);
legend({'Experiment','Hookean'},'interpreter','latex','fontsize',17);
    
function tau = Controller(mdl,Y)
r0 = 2.34e-7;
r1 = 1.6e-8;

alp = [0, 2*pi/3, 4*pi/3];
H   = [r0,r0,r0;...
       r1*sin(alp(1)), r1*sin(alp(2)), r1*sin(alp(3));
       -r1*cos(alp(1)),-r1*cos(alp(2)),-r1*cos(alp(3))];

ii = max(round((mdl.t)/mdl.Tstep),1);
tau =  H*[Y(ii,1);Y(ii,2);Y(ii,3)];
end
    
    