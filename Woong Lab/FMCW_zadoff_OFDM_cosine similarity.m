clear; clc; close all;

% Simulation Parameters
Nzc = 63; % Zadoff-Chu sequence length
rootIdxvec = [8 23 53]; % Root indices for Zadoff-Chu sequences
modOrders = [4, 256, 1024]; % QPSK, 16-QAM, 64-QAM, 256-QAM
snrValues = -5:5:25; % SNR values to evaluate
numIterations = 1e3; % Number of iterations for averaging results
fs = 1e9; % Sampling frequency
numSamples = 1024; % Number of samples for all signals
T_fmcw = numSamples / fs; % FMCW chirp duration
bandwidth=100e6;

% Initialize result storage for averaging
cosSimZC_FMCW_avg = zeros(length(rootIdxvec), length(snrValues));
cosSimOFDM_FMCW_avg = zeros(length(modOrders), length(snrValues));

% Generate FMCW signal
t = (0:numSamples-1) / fs; % Time axis
k = bandwidth / T_fmcw; % FMCW chirp rate
fmcwTx = exp(1i * 2 * pi * (0.5 * k * t.^2)); % FMCW in time domain

% Generate Zadoff-Chu sequences
zcSeqs = cell(1, length(rootIdxvec));
for i = 1:length(rootIdxvec)
    zcSeqs{i} = ifft(zadoffChuSeq(rootIdxvec(i), Nzc), numSamples); % Zero-padded Zadoff-Chu in time domain
end

for iter = 1:numIterations
    % Iterate over SNR values for Zadoff-Chu vs FMCW
    for i = 1:length(rootIdxvec)
        for idx = 1:length(snrValues)
            snr = snrValues(idx);

            % Add AWGN noise
            zcNoisy = awgn(zcSeqs{i}, snr, 'measured');
            fmcwNoisy = awgn(fmcwTx, snr, 'measured');

         
            % Cosine similarity for Zadoff-Chu vs FMCW
            cosSimZC_FMCW_avg(i, idx) = cosSimZC_FMCW_avg(i, idx) + ...
                abs(dot(zcNoisy, fmcwNoisy)) / (norm(zcNoisy) * norm(fmcwNoisy));
        end
    end

    % Generate OFDM signal in time domain and evaluate vs FMCW
    for modIdx = 1:length(modOrders)
        modOrder = modOrders(modIdx);

        % Generate random QAM data
        data = randi([0 modOrder-1], numSamples, 1);
        modData = qammod(data, modOrder, 'UnitAveragePower', true);

        % Convert OFDM to time domain
        ofdmTime = ifft(modData); % IFFT for time domain
       
        for idx = 1:length(snrValues)
            snr = snrValues(idx);

            % Add AWGN noise
            ofdmNoisy = awgn(ofdmTime, snr, 'measured');
            fmcwNoisy = awgn(fmcwTx, snr, 'measured');   

            % Cosine similarity for OFDM vs FMCW
            cosSimOFDM_FMCW_avg(modIdx, idx) = cosSimOFDM_FMCW_avg(modIdx, idx) + ...
                abs(dot(ofdmNoisy, fmcwNoisy)) / (norm(ofdmNoisy) * norm(fmcwNoisy));
        end
    end
end

% Average the results
cosSimZC_FMCW_avg = cosSimZC_FMCW_avg / numIterations; % Average over iterations
cosSimOFDM_FMCW_avg = cosSimOFDM_FMCW_avg / numIterations;

%% Plot results
figure(1);
for i = 1:length(rootIdxvec)
    semilogy(snrValues, cosSimZC_FMCW_avg(i, :), '-o', 'DisplayName', sprintf('Zadoff-Chu (Root Index: %d) vs FMCW', rootIdxvec(i)));
    hold on;
end
for modIdx = 1:length(modOrders)
    semilogy(snrValues, cosSimOFDM_FMCW_avg(modIdx, :), '-o', 'DisplayName', sprintf('OFDM (%d-QAM) vs FMCW', modOrders(modIdx)));
end
xlabel('SNR [dB]');
ylabel('|Cosine Similarity|');
legend('Location', 'best');
grid on;
