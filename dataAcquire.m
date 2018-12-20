clc;
clear all;
close all;

%% INITIALISING
% Load cell calibration properties
b = 0.0018;
m = 6.92;

% Flapping Condition
flapFreq = 1.52;
flapAmp = 15;
delay = 11;

% Begin MATLAB - NI DAQ I/O
ai = daq.createSession('ni');
addAnalogInputChannel(ai,'Dev2', 'ai0', 'Voltage');
ai.Rate = 1000;

disp('Calculating zero load balance...');

[data,time] = ai.startForeground;

zerobal = mean(data);

disp('Begin the flapping motion in Arduino!');
pause(5);

%% DATA COLLECTING
disp('Collecting data...');

ai.DurationInSeconds = 9;

[volt,time] = ai.startForeground;

volt = (volt - zerobal)*1E3;
forceNormal = (volt - b)/m*9.81*1E-3;

%% FFT
L = length(forceNormal);
forceFFT = fft(forceNormal);
spectrumDoubleSide = abs(forceFFT/L);
spectrumSingleSide = spectrumDoubleSide(1:L/2+1);
spectrumSingleSide(2:end-1) = 2*spectrumSingleSide(2:end-1);

f = ai.Rate*(0:(L/2))/L;
plot(f,spectrumSingleSide, 'green');
title('Single-Sided Amplitude Spectrum of X(t)');
xlabel('f (Hz)')
ylabel('|P1(f)|')
grid on
figure;

%% FILTERING & SMOOTHING
% Filter design using 4th order Butterworth filter 
filterOrder = 4;
cutoffFreq = 2;
nyquistFreq = 2*cutoffFreq/ai.Rate;
[z,p] = butter(filterOrder,nyquistFreq);

% Actual filtering process
filterVolt = filter(z,p,volt);
filterForceNormal = filter(z,p,forceNormal);

% Smoothing process using moving average method up to 200 pts forward
smoothForceNormal = smooth(filterForceNormal,200,'moving');
averageForce = mean(smoothForceNormal);

%% CONVERT NORMAL FORCE TO THRUST FORCE
% Create angle of attack array
arrAngleOfAttack = zeros(size(time));
for i = 1:length(arrAngleOfAttack)
    arrAngleOfAttack(i) = flapAmp*cos(2*pi*flapFreq*(time(i) ...
        - (delay*flapAmp/1000))) + 90;
end

forceThrust = zeros(size(smoothForceNormal));
for j = 1:length(smoothForceNormal)
    forceThrust(j) = smoothForceNormal(j)* ...
        sin(deg2rad(arrAngleOfAttack(j)));
end

averageThrust = mean(forceThrust);

%% RESULT DISPLAY
disp('Average Normal Force in mN: ');
disp(averageForce*1000);
disp('Average Net Thrust Force in mN: ');
disp(averageThrust*1000);

%% PLOTTING
plot(time,smoothForceNormal,time,forceNormal);
legend('Filtered', 'Raw');
title('Raw and Filtered Force Data');
xlabel('Time (s)');
ylabel('Normal Force (N)');
grid on;
grid minor;
figure;

plot(time,smoothForceNormal,time,filterForceNormal);
legend('Filtered & Smoothed', 'Filtered Only');
title('Smoothed and Filtered Normal Force Data');
xlabel('Time (s)');
ylabel('Normal Force (N)');
grid on;
grid minor;
figure;

plot(time,forceThrust*1000);
title('Net Thrust Force vs Time');
xlabel('Time (s)');
ylabel('Net Thrust Force (mN)');
grid on;
grid minor;
%% SAVING
evalCase = 21;
save(['Experiment_Result_',num2str(evalCase),'.mat'], ...
    'time','arrAngleOfAttack','forceThrust','smoothForceNormal');