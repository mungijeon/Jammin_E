%% FMCW Radar parameters

function signal_process
    clear;
    clc;
    
    %chatterpub = rospublisher("/topic_r","std_msg/Int32");
    %chattermsg = rosmessage(chatterpub);
    
    % BW = 20MHz, Tsweep = 200us, adc/dac 12bit
    % Fs = 50MHz, Ts = 1/Fs
    % Frequency of operation = 5.8GHz
    % Max Range = 200m
    % Range Resolution = 7.5 m  => c/(2*BW)
    % Max Velocity = 30 m/s
    global range_estimated;
    range_max = 480;    % (m) N*c/(4*BW)
    delta_r = 7.5;      % 7.5,range resolution(m)
    max_vel = 64;  % (m/s), 232(km/h)
    c = 3e8;            % speed of light(m/s)
    
    target_range = 50;
    target_vel = 10;      % 36(km/h)
    
    %% FMCW waverform generation
    
    BW = c/(2*delta_r);
    Tsweep = 2e-4;   
    slope = BW/Tsweep;  % slope of the chirp
    
    fc= 5.8e9;  %carrier freq
                                                   
    M= 128; % NUMBER OF CHIRP per frame
    
    % 샘플링 20MHz*2.5=50MHz 1s에 50MHz, 200us에 1e4Hz(10000 sampling point)
    N= 1e4; % 샘플 많으면 이득, FFT point, 예전값:4096
    % NUMBER OF SAMPLE per chirp, Not 12bit(12bit is representing freq)
    % ampliutude가 높으면 사실상 adc bit 깨지는 건 상관없음, DB 커야함
    Fs = N/Tsweep; % 5e7
    Ts = 1/Fs; 
    
    t=linspace(0,M*Tsweep,N*M); %total time for samples
    
    % Tx, Rx vector
    Tx=zeros(1,length(t));      %Tx signal
    Rx=zeros(1,length(t));      %Rx signal
    beat_signal = zeros(1,length(t));   %beat signal
    
    % t_delay, target_range vector
    r_t=zeros(1,length(t)); %range covered
    roundtrip_delay=zeros(1,length(t)); % time delay
    t_delay=zeros(1,length(t)); 
    
    %% Load signal_Data

   %{
    Tx_data = [];
    Rx_data = [];
    
    for i = 1:256
        filename = sprintf('signal_data%d.txt', i);
        data = load(filename);
        
        if mod(i,2) == 0
            Tx_data = cat(2, Tx_data, data);
        else
            Rx_data = cat(2, Rx_data, data);
        end
    end
    %}
    % beat signal
    %{
    for i=1:length(Tx_data)
        beat_signal(i) = Tx_data(i).*Rx_data(i);
    end
    %}
    
    %% Signal generation  
    
    T = 0;
    
    for i=1:length(t)
       
        % time stamp when constant velocity.
        r_t(i) = target_range + target_vel * t(i);
        roundtrip_delay(i) = 2 * r_t(i) / c; %roundtrip delay
    
        % signal update 
        Tx(i) = cos(2*pi*(fc*(t(i)-T) + slope*((t(i)-T)^2)/2)); %+ randn;
        t_delay = t(i) -T -roundtrip_delay(i);
        Rx(i)  = cos(2*pi*(fc*t_delay + slope*(t_delay^2)/2)) + randn;
        % clutter
        c1_range = 1;
        c2_range = 10;
        c3_range = 100;
        c_t_delay1 = t(i) - T - 2*c1_range/c;
        c_t_delay2 = t(i) - T - 2*c2_range/c;
        c_t_delay3 = t(i) - T - 2*c3_range/c;
        clutter1 = 2*cos(2*pi*(fc*c_t_delay1+ slope*(c_t_delay1^2)/2));
        clutter2 = 1.5*cos(2*pi*(fc*c_t_delay2+ slope*(c_t_delay2^2)/2));
        clutter3 = 0.4*cos(2*pi*(fc*c_t_delay3+ slope*(c_t_delay3^2)/2));
        clutter = clutter1 +clutter2 + clutter3;
        Rx(i) = Rx(i) + clutter;
        % beat signal
        beat_signal(i) = Tx(i).*Rx(i);
        
        if i == N
            T_check = t(i);
        end
    
        if mod(i,N) == 0
            T = T + T_check;
        end
        
    end
    
    %% Windowing(hamming)
    window = hamming(N)'; %flatten 1XN
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
    
    %% Reshape Beat signal
    
    % beat signal FFT (Not windowing)
    beat_signal_no_window = reshape(beat_signal, [N, M]);
    
    % beat signal FFT (windowing)
    beat_signal_window = reshape(windowed_signal, [N, M]);
    
    %% MTI Filter
    num_frames = M; % M = PRI period * 128 = 1 tawsooth * 128 
    
    MTI_filtered_signal = zeros(N , num_frames-1); % 날릴 수 밖에 없었다
    MTI_filtered_signal2 = zeros(N, num_frames-2);
    
    % MTI filter process
    for k = 2:num_frames  % start at second frame (first frame NULL)
        MTI_filtered_signal(:, k-1) = beat_signal_window(:, k) - beat_signal_window(:, k - 1);
    end
    % MTI filter process2(double delay canceller)
    for k = 2:num_frames-1  % start at second frame (first frame NULL)
        MTI_filtered_signal2(:, k-1) = MTI_filtered_signal(:, k) - MTI_filtered_signal(:, k - 1);
    end
    
    
    %% Beat signal FFT
    
    want_to_see_freq = N/4;
    
    % beat signal FFT (Not windowing)
    
    sig_fft_no_window = fft(beat_signal_no_window, N);
    sig_fft_no_window = abs(sig_fft_no_window);
    sig_fft_no_window = sig_fft_no_window ./ max(sig_fft_no_window); % nomalize
    sig_fft_no_window(N/5+1:N,:) = []; % one side of the spectrum.
    
    % beat signal FFT (windowing)
    sig_fft_window = fft(MTI_filtered_signal2, N);
    sig_fft_window = abs(sig_fft_window);
    sig_fft_window = sig_fft_window ./ max(sig_fft_window);  % nomalize
    sig_fft_window(N/5+1:N,:) = []; % one side of the spectrum.
    
    %% Coherent Integration
    % Number of frames to integrate
    frame = M; % discrete PRI period * 128
    
    coherent_integrated_signal = zeros(N/5, 1);
    coherent_integrated_no_window_signal = zeros(N/5,1);
    
    % Coherent integration process
    for k = 1:frame-2
        % Accumulate the signal (coherent integration)
        coherent_integrated_signal = coherent_integrated_signal + sig_fft_window(:, k);
    end
    for k = 1:frame
        coherent_integrated_no_window_signal = coherent_integrated_no_window_signal + sig_fft_no_window(:, k);
    end
    
    %Convert to dB
    sig_dB_no_window = 10*log10(coherent_integrated_no_window_signal);
    sig_dB_window = 10*log10(coherent_integrated_signal);
    
    
    %% Plot
    %{
    x = linspace(1, N/5 , N/5 );
    figure;
    plot(x, sig_dB_window, 'r');
    %plot(x, sig_dB_no_window, 'r', x, sig_dB_window, 'b');
    axis([0 N/2 - 1 -100 100]);
    
    legend('No Window', 'Hamming Window');
    xlabel('Range Bin');
    ylabel('Magnitude (dB)');
    title('FMCW Radar Signal Spectrum');
    %}
    
    %% OS-CFAR
    % Parameters
    num_ref = 4;  % Number of training cells in each dimension
    num_guard = 2;   % Number of guard cells in each dimension
    num_cells = num_ref + num_guard;
    scaling_factor = 1.1; % empirically
    
    CUT_out = zeros(size(sig_dB_window));
    
    % OS-CFAR range
    for i = (num_cells + 1):((N/5)/2 -1 - (num_cells))
            % Extract the CUT(cell under test)
            CUT = (sig_dB_window(i));
            
            j = 1;
            noise_level = 0;
            sort_cells = zeros(size(num_ref*2));
            
            % Extract the Reference cell
            for p = i - (num_cells) : i + (num_cells)
                % In Reference cell
                if (abs(i - p) > num_guard )
                    sort_cells(j) = (sig_dB_window(p));
                    j = j + 1;
                end
            end
            
            % sorting -> Threshold
            sort_cells = sort(sort_cells);
            threshold_cell = sort_cells(length(sort_cells)*3/4);
            threshold = scaling_factor * threshold_cell;
            
            % Compare CUT vs Threshold
            if CUT > threshold
                CUT_out(i) = 1;
            else
                CUT_out(i) = 0;
            end
    end
    
    [~, peak_index] = max(CUT_out); % Find the peak index
    peak_frequency = (peak_index -1) * (Fs / N); % Convertion index -> frequency, quantization err
    range_estimated = peak_frequency * c * Tsweep/(2*BW);

    %chattermsg.Data = range_estimated;
    %send(chatterpub,chattermsg);
    %pause(0.2);
    
    %plot(CUT_out)
    fprintf("Peak index: %f\n",peak_index);
    fprintf('Estimated target range: %.2f meters\n', range_estimated);
end
