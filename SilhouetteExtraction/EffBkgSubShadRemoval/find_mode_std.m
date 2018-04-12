function msd = find_mode_std(st)
%
% Finds the first peak in the histogram of matrix
% st (corresponding to the standard deviation of each pixel), providing an
% estimate of the STD of static pixels in the background.
%
    [c,b] = hist(st(:),512); %histogram

    % Compute gaussian
    g = fspecial('gaussian',[1 10],5);
    nc = wkeep(conv(g,c),length(c),'c');

    % Peak extraction
    [~,i] = max(nc);
    msd = b(i);
end