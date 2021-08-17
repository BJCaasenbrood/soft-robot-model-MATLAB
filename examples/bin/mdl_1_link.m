addpath(genpath('src'));
close all; clear; clc;
%% set number of links
mdl = Model(1);

%% settings
mdl = mdl.setElements(30);
mdl = mdl.setFrequency(60);
mdl = mdl.setLength(0.064);

mdl.tau = @(mdl) Controller(mdl);
%% simulate
mdl = mdl.simulate;

%% post-processing data
l0 = mdl.get('l0');
l  = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx = mdl.q(:,2:3:3*mdl.Nlink);
ky = mdl.q(:,3:3:3*mdl.Nlink);
tauk = mdl.tau(:,2:3);

figure(101);
subplot(3,1,1); plot(mdl.t,l,'linewidth',2); 
ylabel('$l(t)$','interpreter','latex');
subplot(3,1,2); plot(mdl.t,kx,'linewidth',2); 
ylabel('$\kappa_x(t)$','interpreter','latex');
subplot(3,1,3); plot(mdl.t,ky,'linewidth',2); 
ylabel('$\kappa_y(t)$','interpreter','latex');
xlabel('t [s]','interpreter','latex');

%% animate soft robot
figure(102);

for ii = 1:fps(mdl.t,60):length(mdl.t)
    
    figure(102); cla;
    
    mdl.show(mdl.q(ii,:));
    axis equal; axis(0.075*[-1 1 -1 1 0 1]); grid on;
    title(['T = ',num2str(mdl.t(ii),3), '(s)']);
    view(30,30);
    drawnow;
end

function tau = Controller(mdl)
t = mdl.t;
r0 = 10;
r1 = 1;
w = 1;

alp = [0, 2*pi/3, 4*pi/3];
H   = 2.24e-6*[r0,r0,r0;...
               r1*sin(alp(1)), r1*sin(alp(2)), r1*sin(alp(3));
              -r1*cos(alp(1)),-r1*cos(alp(2)),-r1*cos(alp(3))];

tau =  H*([max(sin(w*t),0);...
           max(sin(w*t+2*pi/3),0);...
           max(sin(w*t+4*pi/3),0)]*25e3);

end
    
    