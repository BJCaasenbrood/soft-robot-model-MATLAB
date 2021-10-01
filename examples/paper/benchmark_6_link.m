close all; clear; clc;
load('benchmark_zero');

P0 = P;
T0 = T;

H = [25,50,75,100,125,250];
S = 6*[4,6,10,15,20,30];

TT = [];
EE = [];
EEm = [];
id = 1;

for kk = 1:length(H)
for ll = 1:length(S)   
    
    s = S(ll);
    h = H(kk);
    
    % run model with specific settings
    mdl_6_link_swing
    
    RMS = [];
    for jj = 1:length(P)
        [~,I] = min(abs(P0(:,1) - P(jj,1)));
        
        dr = P0(I,:) - P(jj,:);
        RMS(jj) = sqrt(dr*dr.');
    end
    
    TT(kk,ll) = T/10;
    EE(kk,ll) = mean(RMS);
    EEm(kk,ll) = max(abs(RMS));
    
    %plot(P(:,1),RMS,'linewidth',2); hold on;
    fprintf(' %i out of %i simulations remaining \n',...
        length(H)*length(S) - id, length(H)*length(S));
    id = id + 1;
end
end

clearvars -except TT EE EEm
save('benchmark_results.mat')
