%% FMCW Radar parameters

clear;
clc;

% Tx 신호 바꾸기 -> corr vs index -> DBSCAN, Coherent -> simulink

% BW = 20MHz, Tsweep = 200us, adc/dac 12bit
% Frequency of operation = 5.8GHz
% Max Range = 200m
% Range Resolution = 7.5 m  => c/(2*BW)
% Max Velocity = 30 m/s

range_max = 480;    % (m) N*c/(4*BW)
delta_r = 7.5;      % range resolution(m)
max_vel = 64;  % (m/s), 232(km/h)
c = 3e8;            % speed of light(m/s)

target_range = 50;
target_vel = -20;      % 36(km/h)

%% FMCW waverform generation

BW = c/(2*delta_r);
Tsweep = 2e-4;   
slope = BW/Tsweep;  % slope of the chirp

 
fc= 5.8e9;  %carrier freq
                                               
M=128; % FFT size 

N=4096; % sampling of each chirp
Fs = N/Tsweep;

t=linspace(0,M*Tsweep,N*M); %total time for samples

% Tx, Rx vector
Tx=zeros(1,length(t));      %Tx signal
Rx=zeros(1,length(t));      %Rx signal
beat_signal = zeros(1,length(t));   %beat signal

% t_delay, target_range vector
r_t=zeros(1,length(t)); %range covered
roundtrip_delay=zeros(1,length(t)); % time delay
t_delay=zeros(1,length(t)); 

%% Signal generation  

for i=1:length(t)

   
    % time stamp when constant velocity.
    r_t(i) = target_range + target_vel * t(i);
    roundtrip_delay(i) = 2 * r_t(i) / c; %roundtrip delay
   

    % signal update 
    Tx(i) = cos(2*pi*(fc*t(i) + slope*(t(i)^2)/2)); %+ randn;
    t_delay = t(i) - roundtrip_delay(i);
    Rx(i)  = cos(2*pi*(fc*t_delay + slope*(t_delay^2)/2)); %+ randn;
    %beat signal
    beat_signal(i) = Tx(i).*Rx(i);

    k(i) = fc*t(i) + slope*(t(i)^2)/2;
   
end
plot(Tx)

%% Windowing(hamming)
window = hamming(N)'; %flatten 1X4096
windowed_signal = beat_signal .* repmat(window, 1, M);

%Windowing2(간접 요소곱)
%{
window = hamming(N);

for i = 1:128
    for j = 1:4096
        windowed_signal(i*j) = beat_signal(i*j)*window(j);
    end
end
%}
%% Coherent Integration




%% Beat signal FFT

% beat signal FFT (Not windowing)
beat_signal_no_window = reshape(beat_signal, [N, M]);
sig_fft_no_window = fft(beat_signal_no_window, N);
sig_fft_no_window = abs(sig_fft_no_window);
%sig_fft_no_window = sig_fft_no_window ./ max(sig_fft_no_window); % nomalize
sig_fft_no_window = sig_fft_no_window(1:N/2 - 1); % one side of the spectrum.

% beat signal FFT (windowing)
beat_signal_window = reshape(windowed_signal, [N, M]);
sig_fft_window = fft(beat_signal_window, N);
sig_fft_window = abs(sig_fft_window);
%sig_fft_window = sig_fft_window ./ max(sig_fft_window);  % nomalize
sig_fft_window = sig_fft_window(1:N/2 - 1); % one side of the spectrum.

%Convert to dB
check_no_window = 10*log10(sig_fft_no_window);
check_window = 10*log10(sig_fft_window);

% plot
x = linspace(1, N/2 - 1, N/2 - 1);
figure;
plot(x, check_no_window, 'r', x, check_window, 'b');
axis([0 N/2 - 1 -100 100]);

legend('No Window', 'Hamming Window');
xlabel('Frequency Bin');
ylabel('Magnitude (dB)');
title('FMCW Radar Signal Spectrum');


[~, peak_index] = max(check_window); % Find the peak index
peak_frequency = (peak_index -1) * (Fs / N); % Convertion index -> frequency, quantization err
range_estimated = peak_frequency * c * Tsweep/(2*BW);


fprintf("%f\n",peak_index);

% Display the estimated range
fprintf('Estimated target range: %.2f meters\n', range_estimated);

%% 2D FFT

% 2D FFT for both dimensions.
signal_fft2 = fft2(beat_signal_window, N, M);

% one side of the spectrum
signal_fft2 = signal_fft2(1:N/2,1:M);
signal_fft2 = fftshift (signal_fft2);

% Range Doppler Map
RDM = abs(signal_fft2);
RDM = 10*log10(RDM) ;

doppler_axis = linspace(-63,64,M);
range_axis = linspace(-1023,1024,N/2);

figure,surf(doppler_axis,range_axis,RDM);
title('Amplitude and Range From RDM');
xlabel('Speed');
ylabel('Range');
zlabel('Amplitude');

