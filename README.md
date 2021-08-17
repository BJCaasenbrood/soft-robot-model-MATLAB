[comment]: <https://jsfiddle.net/8ndx694g/> 
# Efficient Soft Robotic Models for (Real-Time) Control Applications -- MATLAB package

The dynamic model for any N-link soft manpulator is given here by the following Lagrangian form:

<img src="https://render.githubusercontent.com/render/math?math=%5Clarge%0A%5Cbegin%7Balign*%7D%0AM(q)%5Cddot%7Bq%7D%20%2B%20C(q%2C%5Cdot%7Bq%7D)%5Cdot%7Bq%7D%20%2B%20G(q)%20%2B%20N(q%2C%5Cdot%7Bq%7D)%20%3D%20%5Ctau%20%2B%20%5Cdelta(t)%0A%5Cend%7Balign*%7D">

where M(.) is the generalized inertia matrix, C(.,.) the Coriolis matrix, G(.) the gravitional forces, N(.,.) a vector of hyper-elastic and visco-elastic contributions, and q(t) the joint variable vector with a particular structure: 

<img src="https://render.githubusercontent.com/render/math?math=%5Clarge%0A%5Cbegin%7Balign*%7D%0Aq%3A%3D(%5Cvarepsilon_1%2C%5Ckappa_%7Bx%2C%5C!1%7D%2C%5Ckappa_%7By%2C%5C!1%7D%2C...%2C%20%5Cvarepsilon_N%2C%5Ckappa_%7Bx%2C%5C!N%7D%2C%5Ckappa_%7By%2C%5C!N%7D)%5E%5Ctop%0A%5Cend%7Balign*%7D">

The assembly and implicit numerical simulation scheme is build into a MATLAB class called: `Model.m`. An example for generating a 8-link soft robot manipulator undergoing free oscillations is given below:

### Natural oscillations of 8-link soft manipulator
```matlab
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
```

<div align="center"> <img src="./fig/mdl_8_link.gif" width="550"> </div> 

### Closed-loop control of 4-link soft manipulator
```matlab
%% generate model class
mdl = Model(4);

%% settings
mdl = mdl.set('Phi0',rotx(pi),'Tsim',15);
mdl = mdl.setElements(60);
mdl = mdl.setFrequency(60);
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
```
<div align="center"> <img src="./fig/mdl_4_closedloop.gif" width="550"> </div> 

### Comparison between hyper-elastic vs. linear materials
The nonlinear stifnesses (for both bending and elongation) are given by:

<img src="https://render.githubusercontent.com/render/math?math=%5Clarge%0A%5Cbegin%7Balign*%7D%0Ak(x)%20%3D%20%5Cunderbrace%7B%5Calpha_1%7D_%7B%5Ctext%7Blinear%7D%7D%20%2B%20%5Cunderbrace%7B%5Calpha_2%5Cleft%5B%5Ctext%7Btanh%7D(%5Calpha_3)%5E2%20-%201%20%5Cright%5D%7D_%7B%5Ctext%7Bnonlinear%7D%7D%0A%5Cend%7Balign*%7D">



```matlab
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

for ii = 1:fps(mdl.t,30):length(mdl.t)
    
    figure(102); cla;
    
    groundplane(0.05);
    mdl.show(Q1(ii,:),col(1));
    mdl.show(Q2(ii,:),col(2));
    
    axis equal; axis(0.4*[-1 1 -1 1 -0.75 1.1]);
    view(0,0); drawnow;
    box on; grid on; set(gca,'linewidth',2.5);

end

function tau = Controller(mdl)
v = [0;sin(mdl.t);0];
tau = 0.02*[v;zeros((mdl.Nlink-1)*3,1)];
end
```

<div align="center"> <img src="./fig/hypervslinear.gif" width="550"> </div> 
