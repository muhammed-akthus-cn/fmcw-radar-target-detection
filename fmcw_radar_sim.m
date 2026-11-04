% Automotive 77GHz FMCW Radar Target Detection
clc; clear; close all;

% 1. Radar Specifications (77 GHz Automotive Standard)
fc = 77e9; % Operating Frequency (77 GHz)
c = physconst('LightSpeed'); % Speed of light
lambda = c/fc;

maxRange = 200; % Maximum range (meters)
rangeRes = 1; % Range resolution (meters)
maxVelocity = 70; % Maximum velocity (m/s) ~ 250 km/h

% Waveform parameters
bw = c / (2 * rangeRes); % Sweep bandwidth
sweepTime = 5.5 * 2 * maxRange / c; % Sweep time
slope = bw / sweepTime; % FMCW slope
fs = 2 * bw; % Sampling frequency

% 2. Define the Target (A car 50m ahead, moving away at 20 m/s)
targetDist = 50; 
targetVel = 20; 
targetRcs = 10; % Radar Cross Section (approximate for a car)

% Create Target and Free Space Channel objects
target = phased.RadarTarget('MeanRCS', targetRcs, 'OperatingFrequency', fc);
channel = phased.FreeSpace('PropagationSpeed', c, 'OperatingFrequency', fc, 'SampleRate', fs, 'TwoWayPropagation', true);

% 3. Radar Hardware Setup
% FIXED: Removed OperatingFrequency from the waveform definition
waveform = phased.FMCWWaveform('SampleRate', fs, 'SweepTime', sweepTime, 'SweepBandwidth', bw);
transmitter = phased.Transmitter('PeakPower', 0.001, 'Gain', 36);
receiver = phased.ReceiverPreamp('Gain', 15, 'NoiseFigure', 4.5, 'SampleRate', fs);

% 4. Simulate the Return Signal (Data Cube Generation)
numSweeps = 128; % Number of chirps per frame
sig = waveform();
txSig = transmitter(sig);

% Preallocate mixed signal matrix
rxData = zeros(length(sig), numSweeps);

for i = 1:numSweeps
    % Update target position based on velocity
    targetPos = [targetDist + targetVel * i * sweepTime; 0; 0];
    targetVelVec = [targetVel; 0; 0];

    % Propagate signal (Transmit -> Channel -> Target -> Channel -> Receive)
    rxSig = channel(txSig, [0;0;0], targetPos, [0;0;0], targetVelVec);
    rxSig = target(rxSig);
    rxSig = receiver(rxSig);

    % Dechirp (Mixer) - multiply received by transmitted signal
    dechirpedSig = dechirp(rxSig, txSig);
    rxData(:, i) = dechirpedSig;
end

% 5. Range-Doppler Processing (2D FFT)
rangeDoppler = phased.RangeDopplerResponse(...
    'RangeMethod', 'FFT', ...
    'DopplerOutput', 'Speed', ...
    'SweepSlope', slope, ...
    'RangeFFTLengthSource', 'Property', 'RangeFFTLength', 1024, ...
    'RangeWindow', 'Hann', ...
    'DopplerFFTLengthSource', 'Property', 'DopplerFFTLength', 256, ...
    'DopplerWindow', 'Hann', ...
    'PropagationSpeed', c, ...
    'OperatingFrequency', fc, ...
    'SampleRate', fs);

% Process the data to extract the raw 2D matrix, range, and speed grids
[resp, rangeGrid, dopplerGrid] = rangeDoppler(rxData);

% 6. Visualization (Manual 3D Surface Plot)
figure('Color', 'w', 'Position', [100, 100, 900, 600]);
% Convert the response magnitude to Decibels (dB) for standard radar viewing
surf(dopplerGrid, rangeGrid, 10*log10(abs(resp))); 
shading interp; % Smooths the color gradient
colormap jet;

title('Automotive 77GHz FMCW Radar: 3D Range-Doppler Map');
xlabel('Velocity (m/s)');
ylabel('Range (m)');
zlabel('Signal Power (dB)');

% Lock the axes to our maximum specifications
axis([-maxVelocity maxVelocity 0 maxRange -inf inf]);
view(45, 45); % 3D isometric view