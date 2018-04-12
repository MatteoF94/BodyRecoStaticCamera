function [midImg,largestSilhIdx] = findFrontImage(imgs,winSize)
%
%  Input(s):
%           imgs - set of sequential images, without duplicates      
%  Output(s):
%           midImg - the image the best represents a target standing facing
%                    directly the camera
%           index - the index of midImg in the input set
%

    numImgs = length(imgs);
    silhWidth = zeros(numImgs,1);
    for i = 1:numImgs
        currImg = imgs{i};
        
        leftIdx = findFirstIdx(currImg,winSize);        
        rightIdx = size(currImg,2) - findFirstIdx(fliplr(currImg),winSize);
        
        silhWidth(i) = rightIdx - leftIdx;
    end
    
    largestSilhIdx = find(silhWidth>=max(silhWidth));
end

function idx = findFirstIdx(currImg,winSize)
    colValues = sum(currImg,1);
    idxs = find(colValues>=1);
    for j = 1:length(idxs)
        window = idxs(j:j + winSize - 1);
        if isempty(find(~colValues(window),1))
            break;
        end
    end
    
    idx = idxs(j);
end