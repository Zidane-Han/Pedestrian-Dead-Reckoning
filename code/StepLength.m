%**************************************************************************
%
%
% Step length estimation using walking frequency and accleration variance
%
% Han Gao
% 10/07/2018
%
%**************************************************************************
clear
close all
clc

load('../SLE/SLdata.mat');

set       = 'ABCDEFGHIJKLM';
StepCount = [42 42 39 58 37 57 43 43 34 38 52 45 47];
dist      = 31.91; % meters

% step length
SL = dist./StepCount;

freq = zeros(1,13);
AV = zeros(1,13); % acceleration variance

for i = 1:13
    eval(['temp = ',set(i),';']);
    interval = (temp(end,1)-temp(1,1))/1000;
    freq(i) = StepCount(i)/interval;
    
    AV(i) = var(sqrt(temp(:,4).^2 + temp(:,5).^2 + temp(:,6).^2));
end

%% LS-Fitting
y = SL';
X = [ones(13,1) freq' AV'];
p = fit([freq' AV'],y,'poly11')


%%
figure(1)
scatter(freq, SL); grid on;
xlabel('Frequency (Hz)');ylabel('Step length (m)');
figure(2)
scatter(AV, SL); grid on;
xlabel('Variance of accelerometer outputs');ylabel('Step length (m)');

