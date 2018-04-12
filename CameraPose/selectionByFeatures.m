function [montagin,mask] = selectionByFeatures(ordImgs, featThresh)
%SELECTIONBYFEATURES Summary of this function goes here
%  The function selects which images are to be kept, depending on the
%  available matching features
    
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
        
        lPts = detectSURFFeatures(lImg,'MetricThreshold',1000,'NumOctaves',1,'NumScaleLevels',12);
        cPts = detectSURFFeatures(cImg,'MetricThreshold',1000,'NumOctaves',1,'NumScaleLevels',12);
        rPts = detectSURFFeatures(rImg,'MetricThreshold',1000,'NumOctaves',1,'NumScaleLevels',12);
        
        [lFeat,~] = extractFeatures(lImg,lPts,'SURFSize',64);
        [cFeat,~] = extractFeatures(cImg,cPts,'SURFSize',64);
        [rFeat,~] = extractFeatures(rImg,rPts,'SURFSize',64);
        
        lcIdxPairs = matchFeatures(lFeat,cFeat,'MatchThreshold',10,'MaxRatio',0.7,'Unique',true);
        crIdxPairs = matchFeatures(cFeat,rFeat,'MatchThreshold',10,'MaxRatio',0.7,'Unique',true);
        
        if length(lcIdxPairs)<featThresh && length(crIdxPairs)<featThresh
            mask(idxC) = 0;
        end
        
        if length(lcIdxPairs)<length(crIdxPairs) && length(lcIdxPairs)<featThresh && length(crIdxPairs)>featThresh
            mask(i) = 0;
        end
        
        if length(lcIdxPairs)>length(crIdxPairs) && length(crIdxPairs)<featThresh && length(lcIdxPairs)>featThresh
            mask(idxR) = 0;
        end
    end
    
    montagin = zeros([size(ordImgs{1}),numImgs],'uint8');
    for i=1:numImgs
        if mask(i)
            montagin(:,:,:,i) = ordImgs{i};
        end
    end
end

