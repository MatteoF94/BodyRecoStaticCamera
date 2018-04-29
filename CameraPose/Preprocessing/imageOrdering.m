function [ordImgs,ordSilhs] = imageOrdering(imagePath,silhPath)
%
%  Input(s): 
%           imagePath - path ponting at the directory containing the
%                       foreground images
%           silhPath - path ponting at the directory containing the
%                       silhouettes 
%  Output(s): 
%           ordImgs - the ordered foregrounds w.r.t. the rotation of the
%                     target
%           ordSilhs - the ordered silhouettes corresponding to the
%                      foregrounds
%

    % Initialize the image datastores for the foregrounds and silhouettes
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

        % Search for the image more similar to the previous one
        imageIDs = retrieveImages(startImg,imageIndex);
   
        currIdx = imageIDs(1);
        imageLocation = imageIndex.ImageLocation{currIdx};
        startImg = imread(imageLocation);
        ordImgs{i} = startImg;
        imds.Files(currIdx) = []; % delete it from the datastore
        
        % Find the corresponding silhouette and remove it from its
        % datastore, using a regular expression to retrieve it by name
        cardImg = str2double(regexp(imageLocation,'(?<=_)\d+(?=.)','match'));
        fullCardImg = sprintf('%.4d',cardImg);
        locations = silhds.Files;
        silhIdx = find(contains(locations,fullCardImg));
        ordSilhs{i} = readimage(silhds,silhIdx);
        silhds.Files(silhIdx) = [];
    end    
    
    % After the iteration, only one image remains in both datasets: the
    % last one
    ordImgs{numImgs} = readimage(imds,1);
    ordSilhs{numImgs} = readimage(silhds,1);
end