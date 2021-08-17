load('openloop_control_data_RTsimuV1.mat');
%load('BrandonSignalTake4.mat');
close all;

i0 = 1;
iN = 9586;%10370;

patm = 1e3;
hpa2pa = 100;

P0 = hpa2pa*(out.P0.Data(i0:end,:) - patm);
P1 = hpa2pa*(out.P1.Data(i0:end,:) - patm);
P2 = hpa2pa*(out.P2.Data(i0:end,:)  - patm);
roll  = out.roll.Data(i0:end,:);
pitch = out.pitch.Data(i0:end,:);
yaw   = out.yaw.Data(i0:end,:);

t = out.P0.Time(i0:end,:)-1;

[~,i0] = min(abs(t)); i0 = i0 + 1;
[~,iN] = min(abs(t-40));

t = t(i0:iN);
P0 = P0(i0:iN,:);
P1 = P1(i0:iN,:);
P2 = P2(i0:iN,:);
roll  = roll(i0:iN);
pitch = pitch(i0:iN);
yaw   = yaw(i0:iN);

%y = @(t,a) hpa2pa*((125*(sin(t + a) + 1).*min(0.05*t,1) + 1100) - patm);

clc; %close all;
nf = 1;
subplot(2,1,1);

kx_ = (pitch)*(pi/180)/0.064;
ky_ = (roll)*(pi/180)/0.064;

plot(t,kx_); hold on;
plot(t,ky_); hold on;

U_ = [smooth(P0(:,1),nf),...
      smooth(P1(:),nf),...
      smooth(P2(:,1),nf)];

%Ud = [y(t(:),0),y(t(:),2*pi/3),y(t(:),4*pi/3)];
 
subplot(2,1,2);
plot(t,U_); hold on;
%plot(t,Ud,'k--');

clearvars -except kx_ ky_ U_

save('control_data_p123.mat')