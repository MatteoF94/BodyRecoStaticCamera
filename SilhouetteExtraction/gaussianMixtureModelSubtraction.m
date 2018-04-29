function silhouettes = gaussianMixtureModelSubtraction(bkgs,imgs,varargin)
%
%  Input(s): 
%           bkgs - the sequence of images representing the scene without
%                  target
%           imgs - the sequence of images representing the scene with the
%                  target
%           varargin - contains the parameters describing number of
%                      gaussian used, minimum background ratio, initial 
%                      variance and learning rate of the detector used
%  Output(s): 
%           silhouettes - the detected silhouettes
%
    
    % Input arguments error checks and parsing
    if length(bkgs) < 1
        error('myfuns:grayscaleSubtraction:TooFewInputs', ...
            'Function requires at least one background image.');
    end
    
    [eta,numGaussians,minBkgRatio,initVariance] = parseInput(varargin);
    
    % Train the detector with the given background images
    numBkgs = length(bkgs);
    detector = vision.ForegroundDetector('NumTrainingFrames',numBkgs,...
        'NumGaussians',numGaussians,'MinimumBackgroundRatio',minBkgRatio,...
        'InitialVariance',initVariance);
    for i=1:numBkgs
        step(detector,bkgs{i});
    end
    
    % Perform foreground detection with a lower learning rate to avoid the
    % target to quickly become part of the background
    detector.LearningRate = eta;
    numImgs = length(imgs);
    silhouettes = cell(numImgs,1);
    for i=1:numImgs
        silhouettes{i} = step(detector,imgs{i});
    end
end

function [eta,numGaussians,minBkgRatio,initVariance] = parseInput(varargin)
%
%  Input(s): 
%           varargin - contains the parameters describing number of
%                      gaussian used, minimum background ratio, initial 
%                      variance and learning rate of the detector used
%  Output(s): 
%           eta - the selected learning rate (default or user defined)
%           numGaussians - the number of gaussians used to model the pixels
%                          (default or user defined)
%           minBkgRatio - the minimum background ratio (default or user
%                         defined)
%           initVariance - the initial variance of the model (default or 
%                          user defined)
%

    numvarargs = length(varargin);
    if numvarargs > 4
        error('myfuns:gaussianMixtureModel:TooManyInputs', ...
            'Function requires at most 4 optional inputs, describing the model parameters.');
    end

    optargs = {0.005 5 0.7 'Auto'}; % default parameters of the object
    for i = 1:numvarargs
        currArg = [varargin{i}{1}];
        switch currArg.param
            case 'eta'
                index = 1;
            case 'numGau'
                index = 2;
            case 'bkgRatio' 
                index = 3;
            case 'var'
                index = 4;
        end
        optargs{index} = currArg.value;
    end
    
    [eta,numGaussians,minBkgRatio,initVariance] = optargs{:};
end