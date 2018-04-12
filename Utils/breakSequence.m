function [seq1,seq2,silh1,silh2] = breakSequence(startImg,sequence,silhouettes)
    imgIdx = detectImageIndex(startImg,sequence);
    numImgs = length(sequence);
    numImgsForSeq = uint8(numImgs/2)-1;
    
    if imgIdx + numImgsForSeq >= numImgs
        leftIdx = mod((imgIdx + imgIdx),numImgs);
        seq1 = [sequence(imgIdx+1:end);sequence(1:leftIdx)];
        silh1 = [silhouettes(imgIdx:end);silhouettes(1:leftIdx)];
        seq2 = flip(sequence(leftIdx+1:imgIdx-1));
        silh2 = flip(silhouettes(leftIdx+1:imgIdx));
    else
        leftIdx = imgIdx + numImgsForSeq;
        seq1 = sequence(imgIdx+1:leftIdx);
        silh1 = silhouettes(imgIdx+1:leftIdx);
        seq2 = flip([sequence(leftIdx+1:end);sequence(1:imgIdx-1)]);
        silh2 = flip([silhouettes(leftIdx+1:end);silhouettes(1:imgIdx-1)]);
    end
end

function index = detectImageIndex(startImg,sequence)
    % Exploits Eearth Mover Distance
    startHistR = imhist(startImg(:,:,1));
    startHistG = imhist(startImg(:,:,2));
    startHistB = imhist(startImg(:,:,3));
    
    numImgs = length(sequence);
    distances = zeros(numImgs,3);
    for i = 1:numImgs
        currImg = sequence{i};
        histR = imhist(currImg(:,:,1));
        histG = imhist(currImg(:,:,2));
        histB = imhist(currImg(:,:,3));
        
        distR = emd(startHistR,histR);
        distG = emd(startHistG,histG);
        distB = emd(startHistB,histB);
        
        distances(i,:) = [distR,distG,distB];
    end
    
    distMetric = zeros(numImgs,1);
    for i = 1:numImgs
        distMetric(i) = sum(distances(i,:));
    end
    
    index = find(distMetric == min(distMetric));
end