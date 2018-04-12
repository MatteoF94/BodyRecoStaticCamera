function silhouettes = gaussianMixtureModelSubtraction(bkgs,imgs,varargin)
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