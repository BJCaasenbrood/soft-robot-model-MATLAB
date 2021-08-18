%% set number of links
mdl = Model(4);

%% throwing parameters
trel = 7.7;
flr  = -0.35;

%% settings
mdl = mdl.set('Phi0',rotx(pi),'Tsim',15);
mdl = mdl.setElements(40);
mdl = mdl.setFrequency(50);
mdl = mdl.setLength(0.065);
%mdl = mdl.set('db',1.5e-5);

mdl.tau = @(mdl) Controller(mdl);

%% simulate with initial conditions
mdl = mdl.simulate;

%% simulating throwing ball
[~,I]    = min(abs(mdl.t-trel));
[p0, v0] = mdl.computeEndEffector(mdl.q(I,:),mdl.dq(I,:));

vn = norm(v0(4:end));
th = atan2(v0(6),v0(4));

[~,Y] = ode45(@(t,x) BallODE(t,x,flr), mdl.t(I:end) - mdl.t(I),...
    [p0(1);p0(3);vn*cos(th);vn*sin(th)]); 

%% post-processing data
l0  = mdl.get('l0');
l   = mean(l0)*(1+mdl.q(:,1:3:3*mdl.Nlink));
kx  = mdl.q(:,2:3:3*mdl.Nlink);
ky  = mdl.q(:,3:3:3*mdl.Nlink);
dkx = mdl.dq(:,2:3:3*mdl.Nlink);
dky = mdl.dq(:,2:3:3*mdl.Nlink);

f = figure(101); f.Name = 'Bishop parameters';
subplot(3,1,1); plot(mdl.t,l,'linewidth',2); 
ylabel('$l(t)$','interpreter','latex','fontsize',20);
subplot(3,1,2); plot(mdl.t,kx,'linewidth',2); 
ylabel('$\kappa_x(t)$','interpreter','latex','fontsize',20);
subplot(3,1,3); plot(mdl.t,ky,'linewidth',2); 
ylabel('$\kappa_y(t)$','interpreter','latex','fontsize',20);
xlabel('time $t$ [s]','interpreter','latex','fontsize',20);


%% plot phase portaits
f = figure(102); f.Name = 'Phase portrait';
subplot(2,2,1); plot(kx(:,1),dkx(:,1),'linewidth',2); 
subplot(2,2,2); plot(kx(:,2),dkx(:,2),'linewidth',2); 
subplot(2,2,3); plot(kx(:,3),dkx(:,3),'linewidth',2); 
subplot(2,2,4); plot(kx(:,4),dkx(:,4),'linewidth',2); 

pause
%% play animation soft robot
f  = figure(103);
Q  = mdl.q;
jj = 1;

% animation for-loop
for ii = 1:fps(mdl.t,225):length(mdl.t)
    
    figure(103); cla;
    % plot soft robot
    mdl.show(Q(ii,:));
    
    % plot throwing object
    if mdl.t(ii) > trel
        plot3(Y(1:jj,1),Y(1:jj,1)*0,Y(1:jj,2),'r-','linewidth',2);
        plot3(Y(jj,1),Y(jj,1)*0,Y(jj,2),'r.','markersize',22);
        jj = jj + fps(mdl.t,120);
    end
    
    axis equal; axis(0.4*[-0.75 1.5 -1 1 -1 0.5]);
    plot3([-0.3,1.2],[0;0],[-0.35,-0.35],'k-');
    f.Name = [' Time =',num2str(mdl.t(ii),3)];
    view(0,0)
    drawnow;
    
end


%% model-based controller
function tau = Controller(mdl)
v1  = [0;sin(7.30*mdl.t);0];
tau = 0.05*[zeros((mdl.Nlink-1)*3,1);v1];
end

% ball model
function dx = BallODE(~,x,flr)
g  = 9.81;
nu = 0.5;

dx(1,1) = x(3);
dx(2,1) = x(4);
dx(3,1) = -nu*x(3);
dx(4,1) = -g-nu*x(4);

if x(2) <  flr
    dx(1,1) = 0;
    dx(2,1) = 0;
    dx(3,1) = 0;
    dx(4,1) = 0;
end

end
    
    