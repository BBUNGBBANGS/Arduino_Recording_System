%%
%     PURPOSE: Compare fft with dft loop for an audio file
%  AUDIO FILE: Use a public domain birdcall from xeno-canto.org
%      RESULT: Produce 3 plots: 1) Time domain, 2) FFT, and 3) DFT
%   DEVELOPER: http://www.biophysicslab.com/portfolio/matlab-projects/
%        DATE: June 6, 2020
%
%  REFERENCES:
%   "Signal Processing Problems" hosted on Udemy.com
%       by Dr. Mike X Cohen, sigprocMXC_SpectBirdcall.m
%   "Master the Fourier transform and its applications" hosted on Udemy.com 
%       by Dr. Mike X Cohen, Fourier_DTFT.m
%
%%
% Load in birdcall (source: https://www.xeno-canto.org/403881).
[bc,fs] = audioread('F:\0007.WAV');
% Configuration params for Fourier transforms.
freq_range = [0 8000]; % Hz
dft_loop_reduction = 100; % set to 1 for no reduction (Hint: takes a long time)
% let's hear it!
% soundsc(bc,fs)
n = length(bc);
hz = linspace(0,fs/2,floor(n/2)+1);
% Smooth birdcall audio audio for fft & dft
signal = detrend(bc(:,1))'; % transpose here for DFT loop later
% Create a time vector based on the data sampling rate.
timevec = (0:n-1)/fs;
% Plot the data from the two audio file channels.
figure(2), clf
subplot(311)
% Include a small offset for left and right audio channels.
plot(timevec,bsxfun(@plus,bc,[.2 0]))
xlabel('Time (sec.)')
title('Time domain')
set(gca,'ytick',[],'xlim',timevec([1 end]))
%% Compute & plot the power spectrum using MATLAB fft function.
bcpow_fft = abs(fft( signal )/n).^2;
subplot(312)
plot(hz,bcpow_fft(1:length(hz)),'linew',2)
xlabel('Frequency (Hz)')
ylabel('Power');
title(['Frequency domain using FFT with ' num2str(length(bcpow_fft)) ' points']);
set(gca,'xlim',freq_range)
% Make fft and dft y-limits be the same for comparison.
ylim_fft = get(gca,'ylim'); 
%% Compute & plot the power spectrum using DFT loop.
%%fourTime = (0:n-1)/n;
%%fCoefs   = zeros(size(signal));
%%h = waitbar(0,'Please wait for DFT loop...');
% DFT loop is very inefficient, it is used here for demonstration only.
%%for fi=1:dft_loop_reduction:n
    
    % Create complex sine wave.
%%    csw = exp( -1i*2*pi*(fi-1)*fourTime );
    
    % Compute dot product between sine wave and signal (Fourier coefficients).
%%    fCoefs(fi) = sum( signal.*csw );
    
    % GUI to show progress for long calculation times.
%%    waitbar(fi/n)
%%end
%%close(h)
%%bcpow_dft = abs(fCoefs / n).^2;
%%subplot(313)
%%plot(hz,bcpow_dft(1:length(hz)),'linew',2)
%%xlabel('Frequency (Hz)')
%%ylabel('Power');
%%title(['Frequency domain using DFT loop with ' ...
%%    num2str(floor(length(bcpow_dft)/dft_loop_reduction)) ' points']);
%%set(gca,'xlim',freq_range)
%%set(gca,'ylim',ylim_fft)
%% done.