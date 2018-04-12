function matchedSilhs = matchSilhouettes(imgs,silhs)
%
%
% NOT USED
%
%
    numImgs = length(imgs);
    matchedSilhs = cell(numImgs,1);
    
    for i = 1:numImgs
        currImg = imgs{i};
        numSilhs = length(silhs);
        numZeros = zeros(1,numSilhs);
        
        for j = 1:numSilhs
            currSilh = silhs{j};
            currDiff = bsxfun(@times,currImg,cast(~currSilh,class(currImg)));
            numZeros(j) = sum(sum(rgb2gray(currDiff)~=0));
        end
        
        matchedSilhs{i} = silhs{numZeros == min(numZeros)};
        silhs(numZeros == min(numZeros)) = [];
        showMontage(matchedSilhs);
    end
end