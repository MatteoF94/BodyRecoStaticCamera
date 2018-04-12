function [w,msd,k,st] = backgroundTM(bkgs)
%
%  Model generation: explores the x84 outlier rule during training to
%  estimate the standard deviation at each pixel.
%
%  Input(s): 
%           input_bkgs - images of the background used to train the model
%  Output(s): 
%           w - the background model using the metrically-trimmed mean
%           msd - mode of estimated standard deviations (not used in  the
%           paper)
%           k - the threshold for foreground detection (set to the default value 3)
%           st - the estimated standard deviation based on the MAD
%

    % Input arguments error checks
    if length(bkgs) < 1
           error('myfuns:backgroundTM:TooFewInputs', ...
               'Function requires at least one background image.');
    end

    % Creates a stack of "frames", using the input images
    nframes = length(bkgs);
    sizes = [size(rgb2gray(bkgs{1})),nframes];
    x = zeros(sizes,'uint8');
    for i = 1:nframes
        y = bkgs{i};  %reads frame i from video sequence or camera, and returns an image y (must write!)
        if size(y,3)>1
            y = double(rgb2gray(y));
        end

        % Stacks all grayscale images in a 3D array x
        x(:,:,i) = y;
    end

    % Computes the background model
    [w,msd,k,st] = new_background_metrically_trimmed(x);
end
