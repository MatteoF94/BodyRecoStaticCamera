function [cleanImgs,cleanSilhs] = wipeDuplicates(imgs,silhs,varargin)
%
%  Input(s):
%           imgs - set of foregrounds to clean from duplicates
%           silhs - set of silhouettes to clean from duplicates
%           varargin(threshold) - optional argument that allows the user to select
%                                 the sensitivity of comparison between images, to 
%                                 determine if they represent the same scene, default
%                                 value is 1.
%  Output(s):
%           cleanImgs - the input set of images polished of duplicates
%           cleanSilhs - the input set of silhouettes polished of duplicates 
%
  
    % Select the threshold value for the comparison sensitivity.
    if isempty(varargin)
        threshold = 1;
    else
        threshold = varargin{1};
    end
    
    % We need an array to represent the fact that an image is a duplicate
    % or not, not interested in the "of what image" aspect.
    numImgs = length(imgs);
    flags = ones([1,numImgs],'logical');
    
    % Cycle through all the images and select the duplicates, even
    % confronting the last and the first one.
    for i = 1:numImgs
        if i == numImgs
            idx = 1;
        else
            idx = i+1;
        end
        
        % Two images are the same if the average of the feature distances
        % is too low or if their colorimatric distance is almost null
        isSame = checkSameImage(imgs{i},imgs{idx},threshold);
        distRGB = crossBinPairEMD(imgs{i},imgs{idx});
        isEMD = abs(distRGB(1))<20 && abs(distRGB(2))<20 && abs(distRGB(3))<20;
        if isSame || isEMD
            if i ~= numImgs
                flags(idx) = 0;
            else
                flags(i) = 0;
            end
        end
    end
    
    % We exploit the flags array to select the non-duplicates as output.
    numCleanImgs = sum(flags);
    cleanImgs = cell(numCleanImgs,1);
    cleanSilhs = cell(numCleanImgs,1);
    k = 1;
    for i = 1:numImgs
        if flags(i)
            cleanImgs{k} = imgs{i};
            cleanSilhs{k} = silhs{i};
            k = k+1;
        end
    end
end

% ----------------------------------------------------------------------- %

function isSame = checkSameImage(startImg,selectedImg,threshold)
%
%  Input(s):
%           startImg - the "original" image
%           selectedImg - the image to be checked if equal to the previous
%           threshold - optional argument that allows the user to select
%                       the sensitivity of comparison between images, to 
%                       determine if they represent the same scene                           
%  Output(s):
%           isSame - 1 if selectedImg is equal to startImg, 0 otherwise
%

    % Detect SURF features in both the images and match them, without
    % having much care in the selection parameters tuning: if two images
    % are the same, in any condition the extracted features should be the
    % same.
    startPoints = detectSURFFeatures(rgb2gray(startImg));
    [firstFeatures,validFirstPts] = extractFeatures(rgb2gray(startImg),startPoints);
    currPoints = detectSURFFeatures(rgb2gray(selectedImg));
    [currFeatures,validCurrPts] = extractFeatures(rgb2gray(selectedImg),currPoints);
    
    indexPairs = matchFeatures(firstFeatures,currFeatures,'Unique',true,'MatchThreshold',50,'MaxRatio',0.5);
    
    matchingStartPts = validFirstPts(indexPairs(:,1),:);
    matchingCurrPts = validCurrPts(indexPairs(:,2),:);
    
    % It is straightforward to check if two images are the same: the
    % distances between matched features should be very small. Depending on
    % how much precise we want to be, the user has the option to select
    % manually the threshold on the median (to avoid outlier) of the
    % distances.
    distances = zeros(1,length(matchingStartPts));
    for i=1:length(matchingStartPts)
        distances(i) = pdist2(matchingStartPts(i).Location,matchingCurrPts(i).Location,'Euclidean');
    end
    
    if median(distances) <= threshold && median(distances) >= -threshold
        isSame = 1;
    else
        isSame = 0;
    end
end