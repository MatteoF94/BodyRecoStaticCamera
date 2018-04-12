function [ordImgs,ordSilhs] = imageOrdering(imagePath,silhPath)
%IMAGEORDERING Summary of this function goes here
%   Detailed explanation goes here
    imds = imageDatastore(imagePath);
    silhds = imageDatastore(silhPath);
    numImgs = length(imds.Files);
    ordImgs = cell(numImgs,1);
    ordSilhs = cell(numImgs,1);
    
    % Select the first image in the dataset to begin the search, removing
    % it from the pool
    currIdx = 1;
    startImg = readimage(imds,currIdx);
    ordSilhs{currIdx} = readimage(silhds,currIdx);
    ordImgs{currIdx} = startImg;
    
    imds.Files(currIdx) = [];
    silhds.Files(currIdx) = [];

    for i = 2:numImgs-1
        imageIndex = indexImages(imds,'Verbose',0,'SaveFeatureLocation',0);

        % Begin the search iteration 
        imageIDs = retrieveImages(startImg,imageIndex);
   
        currIdx = imageIDs(1);
        imageLocation = imageIndex.ImageLocation{currIdx};
        startImg = imread(imageLocation);
        ordImgs{i} = startImg;
        imds.Files(currIdx) = [];
        
        cardImg = str2double(regexp(imageLocation,'(?<=_)\d+(?=.)','match'));
        fullCardImg = sprintf('%.4d',cardImg);
        locations = silhds.Files;
        silhIdx = find(contains(locations,fullCardImg));
        ordSilhs{i} = readimage(silhds,silhIdx);
        silhds.Files(silhIdx) = [];
    end    
    
    ordImgs{numImgs} = readimage(imds,1);
    ordSilhs{numImgs} = readimage(silhds,1);
end

%
% TODO: when selecting currIdx at 21, check wether the sense is correct,
% selecting common features and making the average (left or right from the
% old one?)
%

