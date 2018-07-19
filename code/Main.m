clear
close all
clc

% SL = 0.2844 + 0.2231*frequency + 0.0426*AV

load ../context_demo/four/SensorLog.mat
trial = SensorLog;
% trial = L;

%% preprocessing
yaw = trial(:,13);
zoneA = find(yaw>=0 & yaw<90);
zoneB = find(yaw>=90 & yaw<180);
zoneC = find(yaw>=180 & yaw<270);
zoneD = find(yaw>=270 & yaw<360);
% range of sin -->(-90,+90)
% range of cos -->(0,+180)
yaw(zoneA,2:3) = [yaw(zoneA,1) yaw(zoneA,1)];
yaw(zoneB,2:3) = [180-yaw(zoneB,1) yaw(zoneB,1)];
yaw(zoneC,2:3) = [180-yaw(zoneC,1) 360-yaw(zoneC,1)];
yaw(zoneD,2:3) = [yaw(zoneD,1)-360 360-yaw(zoneD,1)];


acc = sqrt(trial(:,4).^2 + trial(:,5).^2 + trial(:,6).^2);
acc = acc-mean(acc);

%% BANDPASS FILTER
fs=100;
f1=0.75;               % cuttoff low frequency to get rid of baseline wander
f2=2.75;                 % cuttoff frequency to discard high frequency noise
Wn=[f1 f2]/(fs/2);    % cutt off frequency based on fs
N = 4;                % order of 3 less processing

[a,b] = butter(N,Wn); % bandpass filtering
bandPass = filtfilt(a,b,acc);
% bandPass = bandPass/ max(abs(bandPass));

%% find peaks
% Find signal peaks - peaks under a threshold value are considered as noise.
[PkValue, PeakLocation] = findpeaks(bandPass, 'MINPEAKHEIGHT', 0.25);

%% time interval (steps between 0.4s and 2s)
PkValue(:,2) = trial(PeakLocation,1);
PkValue(2:end,2) = PkValue(2:end,2)-PkValue(1:end-1,2);
index = find(PkValue(:,2)<400);
if isempty(index) == 0
    pos_del = [];
    for k = 1:length(index)
        temp = index(k); % position of the suspicious samples
        if PkValue(temp,1) <= PkValue(temp-1,1)
            pos_del = [pos_del; temp];
        else
            pos_del = [pos_del; temp-1];
        end
    end
    PeakLocation(pos_del) = [];
    PkValue(pos_del,:) = [];
end
StepCount = length(PeakLocation); % step number

%% position update
PositionX = zeros(StepCount, 1);
PositionY = zeros(StepCount, 1);
distance = 0;
for k = 1:StepCount-1
    pos_start = PeakLocation(k);
    pos_end = PeakLocation(k+1);
    % orientation (yaw)
    YawSin = mean(yaw(pos_start:pos_end,2));
    YawCos = mean(yaw(pos_start:pos_end,3));
    % step length estimation
    % SL = 0.2844 + 0.2231*frequency + 0.0426*AV
    StepFreq = 1000/PkValue(k+1,2);
    StepAV = var(acc(pos_start:pos_end));
    StepLength = 0.2844 + 0.2231*StepFreq + 0.0426*StepAV;
    distance = distance + StepLength;
    
    % position update
    PositionX(k+1) = PositionX(k) + StepLength * cos(deg2rad(YawCos));
    PositionY(k+1) = PositionY(k) + StepLength * sin(deg2rad(YawSin));
end
distance

%% figures
figure(1)
plot(bandPass)
figure(2)
scatter(PositionY,PositionX); grid on; % north-up
% x & y on the same scale
xlim([-50 50])
ylim([0 100])

