function [montagin,mask] = selectionByFeatures(ordImgs, featThresh)
%
%  Input(s): 
%           ordImgs - sequence of ordered and without duplicates of images 
%                     w.r.t. the rotation of the target
%           featThresh - threshold for which two subsequent images are
%                        selected: if not enough features are found, one of
%                        them is discarded
%  Output(s): 
%           montagin - montage of the ordered images, blacked in the spots
%                      where a not accepted image is placed
%           mask - a binary list indicating which image is accettable and
%                    which not
    
    % Since the images are ordered, we need only to check them three by
    % three
    numImgs = length(ordImgs);
    mask = ones(1,numImgs,'logical');
    
    for i = 1:numImgs-1
        if i == numImgs-1
            idxR = i-numImgs+2;
        else
            idxR = i+2;
        end
        idxC = i+1;
        lImg = rgb2gray(ordImgs{i});
        cImg = rgb2gray(ordImgs{idxC});
        rImg = rgb2gray(ordImgs{idxR});
        
        % Detect and extract SURF features in the images 
        lPts = detectSURFFeatures(lImg,'MetricThreshold',500,'NumOctaves',12,'NumScaleLevels',12);
        cPts = detectSURFFeatures(cImg,'MetricThreshold',500,'NumOctaves',12,'NumScaleLevels',12);
        rPts = detectSURFFeatures(rImg,'MetricThreshold',500,'NumOctaves',12,'NumScaleLevels',12);
        
        [lFeat,~] = extractFeatures(lImg,lPts,'SURFSize',64);
        [cFeat,~] = extractFeatures(cImg,cPts,'SURFSize',64);
        [rFeat,~] = extractFeatures(rImg,rPts,'SURFSize',64);
        
        % Find matching features for each pair of images: left-central and
        % central-right
        lcIdxPairs = matchFeatures(lFeat,cFeat,'MatchThreshold',10,'MaxRatio',0.7,'Unique',true);
        crIdxPairs = matchFeatures(cFeat,rFeat,'MatchThreshold',10,'MaxRatio',0.7,'Unique',true);
        
        % If we don't find enough matchings in each pair, the bad image is
        % the central one
        if length(lcIdxPairs)<featThresh && length(crIdxPairs)<featThresh
            mask(idxC) = 0;
        end
        
        % If we don't find enough features in the left-central pair, the
        % bad image is the left one
        if length(lcIdxPairs)<length(crIdxPairs) && length(lcIdxPairs)<featThresh && length(crIdxPairs)>featThresh
            mask(i) = 0;
        end
        
        % If we don't find enough features in the central-right pair, the
        % bad image is the right one
        if length(lcIdxPairs)>length(crIdxPairs) && length(crIdxPairs)<featThresh && length(lcIdxPairs)>featThresh
            mask(idxR) = 0;
        end
    end
    
    % Create the blacked montage
    montagin = zeros([size(ordImgs{1}),numImgs],'uint8');
    for i=1:numImgs
        if mask(i)
            montagin(:,:,:,i) = ordImgs{i};
        end
    end
end

