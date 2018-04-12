function silhouettes = grayscaleSubtraction(bkgs,imgs,varargin)
%
%  Input(s):
%           bkgs - images of the background, used to average over time
%           imgs - images in which we want to extract a target silhouette
%           varargin - optional arguments describing the user inserted
%                      thresholds. Each is a struct with two fields: 'ch',
%                      the color channel considered ('r','g' or 'b'), and
%                      'value', containing the threshold value for the
%                      specific channel
%  Output(s):
%           silhouettes - the extracted silhouettes from the input images
%

    % Input arguments error checks and parsing
    if length(bkgs) < 1
        error('myfuns:grayscaleSubtraction:TooFewInputs', ...
            'Function requires at least one background image.');
    end
    
    [rThresh, gThresh, bThresh] = parseInput(varargin);

    % To model the background we need to extract separaterly the three
    % color channel for all the available background images and compute the
    % average
    numBkgs = length(bkgs);
    [h,l,~] = size(bkgs{1});
    rBkg = zeros(h,l,'uint32');
    gBkg = zeros(h,l,'uint32');
    bBkg = zeros(h,l,'uint32');
    for i = 1:numBkgs
        rBkg = rBkg + uint32(bkgs{i}(:,:,1));
        gBkg = gBkg + uint32(bkgs{i}(:,:,2));
        bBkg = bBkg + uint32(bkgs{i}(:,:,3));
    end
    
    rBkg = uint8(rBkg/numBkgs); 
    gBkg = uint8(gBkg/numBkgs); 
    bBkg = uint8(bBkg/numBkgs); 
    
    % A pixel of an image belongs to the foreground only if the absolute
    % difference between it and the corresponding pixel in the modeled
    % background is above all the used thresholds, one for each color
    % channel
    numImgs = length(imgs);
    silhouettes = cell(numImgs,1);
    for i = 1:numImgs
        currImg = imgs{i};
        rMask = abs(currImg(:,:,1) - rBkg) > rThresh;
        gMask = abs(currImg(:,:,2) - gBkg) > gThresh;
        bMask = abs(currImg(:,:,3) - bBkg) > bThresh;
        
        silhouettes{i} = rMask+gMask+bMask;
    end
end

% ----------------------------------------------------------------------- %

function [r,g,b] = parseInput(varargin)
    %
    %  Input(s):
    %           varargin - the optional arguments of the calling function
    %  Output(s):
    %           r - red channel threshold (default or selected)
    %           g - green channel threshold (default or selected)
    %           b - blue channel threshold (default or selected)
    %
    
    numvarargs = length(varargin);
    if numvarargs > 3
        error('myfuns:grayscaleSubtraction:TooManyInputs', ...
            'Function requires at most 3 optional inputs, describing color channels thresholds.');
    end

    optargs = {4 4 4}; % set the default thresholds
    for i = 1:numvarargs
        currArg = [varargin{i}{1}];
        switch currArg.ch
            case 'r'
                index = 1;
            case 'g'
                index = 2;
            case 'b' 
                index = 3;
        end
        optargs{index} = currArg.value;
    end
    
    [r,g,b] = optargs{:};
end