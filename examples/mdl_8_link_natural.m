%% generate model class
mdl = Model(8);

%% settings
mdl = mdl.set('Tsim',10);
mdl = mdl.setElements(50);
mdl = mdl.setFrequency(50);
mdl = mdl.setLength(0.03);

%% simulate with non-zero initial conditions
mdl.q0(2:3:end) = 5;
mdl = mdl.simulate;

%% show simulation
figure(102);

for ii = 1:fps(mdl.t,30):length(mdl.t)
    figure(102); cla;
    
    mdl.show(mdl.q(ii,:),col(1));
    axis equal; axis(0.25*[-0.4 1.25 -1 1 -1 1]);
    view(0,0); grid on; box on; 
    drawnow(); 
end