%% set number of links
mdl = Model(6);

%% settings
mdl = mdl.set('Tsim',10);
mdl = mdl.setElements(60);
mdl = mdl.setFrequency(50);

%% setting properties
mdl = mdl.setLength(0.025);
mdl = mdl.set('kb',[0.425, 0.395, -0.21]);
mdl = mdl.setMass(0.05);
mdl = mdl.setRadius(0.05);
mdl = mdl.setDamping(1e-5);

%% set controller
mdl.tau = @(mdl) Controller(mdl);

%% simulate with initial conditions
mdl.q0(2:3:end) = 1;
mdl = mdl.simulate;

%% post-processing data
t  = mdl.t;
l0 = mdl.get('l0');
l  = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
ee = mdl.q(:,1:3:3*mdl.Nlink);
kx = mdl.q(:,2:3:3*mdl.Nlink);
ky = mdl.q(:,3:3:3*mdl.Nlink);
dkx = mdl.dq(:,2:3:3*mdl.Nlink);
dky = mdl.dq(:,2:3:3*mdl.Nlink);

f = figure(101); f.Name = 'Bishop parameters';
subplot(2,1,1); set(gca,'linewidth',1.5);
plot(t,ee,'linewidth',2.5); hold on;
ylabel('$\varepsilon$ (-)','interpreter','latex','fontsize',18);
colororder(repmat(linspace(0.7,0.1,6).',1,3))

grid on;
subplot(2,1,2); set(gca,'linewidth',1.5);
plot(t,kx,'linewidth',2.5); hold on;
ylabel('$\kappa$ (m$^{-1}$)','interpreter','latex','fontsize',18); 
axis([0 mdl.t(end) -110 110]); grid on;
subplot(2,1,2); set(gca,'linewidth',1.5);
colororder(repmat(linspace(0.7,0.1,6).',1,3))
xlabel('time (s)','interpreter','latex','fontsize',18);

%% play animation soft robot
f = figure(103); 
Q = mdl.q;

%% recover trajectory end effector
P = zeros(length(mdl.t),3);
for ii = 1:length(mdl.t)
    [p0, ~] = mdl.computeEndEffector(mdl.q(ii,:),mdl.dq(ii,:));
    P(ii,:) = p0.';
end

%% animate soft robot
for ii = 1:fps(mdl.t,20):length(mdl.t)
    figure(103); cla;
    
    mdl.show(mdl.q(ii,:),gcol(3));
    
    groundplane(0.02);
    axis equal;
    axis(0.15*[-1 1 -1 1 -0.75 1]);
    view(0,0); drawnow;
end

%% open-loop controller
function tau = Controller(mdl)
A = 0.02;
w = pi;
v = [0;sin(w*mdl.t);0];
tau = A*[v;zeros((mdl.Nlink-1)*3,1)];
end
    

    