function [w,msd,k,st]=new_background_metrically_trimmed(x)
%
% Based on a sequence of grayscale frames  x (height x width x number of
% frames), computes the background as the metrically trimmed mean for each
% pixel, and estimates the standard deviation based on the MAD estimate
%
    % Median
    [w,med]=metrically_trimed_mean(x,.3);
    s = size(x);
    
    % Compute MAD
    madd = median(abs(repmat(med,[1 1 s(3)])-double(x)),3); 
    % md = median(madd(:));
    madd(madd==0) = .5; % Defines a minimum value of 0.5 for the MAD
    st = 1.4826*madd; % estimates the standard deviation
    msd = find_mode_std(st(w>20 & w<230)); % computes the mode of the standard deviation (excludes regions that are very dark or very bright)
    k = 3; % default value for the standard deviation threshold
end