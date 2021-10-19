%% generate model class
mdl = Model(4);

%% settings
mdl = mdl.set('Phi0',rotx(pi),'Tsim',15);
mdl = mdl.setElements(60);
mdl = mdl.setFrequency(30);
mdl = mdl.setLength(0.065);

%% set model-based controller (computed torque control, see below)
qd  = [0;20;10;0;-20;0;0;0;30;0;-40;0]; 
mdl = mdl.setControl( @(mdl) Controller(mdl,qd) );

%% simulate with zero initial conditions
mdl = mdl.simulate;

%% show simulation
figure(102);
Qd = [0;20;10;0;-20;0;0;0;30;0;-40;0].';

for ii = 1:fps(mdl.t,12):length(mdl.t)
    figure(102); cla;
    mdl.show(mdl.q(ii,:),col(1));
    mdl.show(Qd,col(2));
    
    groundplane(0.02);
    axis equal; axis(0.2*[-0.75 0.75 -0.75 0.75 -1.5 0.1]);
    view(30,30); grid on; box on;  drawnow(); 
end

%% model-based controller
function tau = Controller(mdl,qd)
  Kp = 1e-4*eye(12);
  Kd = 5e-5*eye(12);
  tau = mdl.G + mdl.K*(mdl.q) - Kp*(mdl.q - qd) - Kd*(mdl.dq);
end
