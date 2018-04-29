function [seq1,seq2,silh1,silh2] = breakSequence(startImg,sequence,silhouettes,isAntisym)
%
%  Input(s): 
%           startImg - image that breaks the sequence
%           sequence - complete set of images
%           silhouettes - complete set of silhouettes
%           isAntisym - parameters that determines if we need to break the
%                       sequence from start to end or evenly, treating it 
%                       as a circular list of images
%  Output(s): 
%           seq1 - first subsequence (including breaking image)
%           seq2 - second subsequence (including breaking image)
%           silh1 - first subsequence of silhouettes (including the silhouette
%                   correspondant to the breaking image)
%           silh2 - second subsequence of silhouettes (including the silhouette
%                   correspondant to the breaking image)
%
    
    % First we need to find the breaking image in the sequence
    imgIdx = detectImageIndex(startImg,sequence);
    
    numImgs = length(sequence);
    
    % Consider the sequence from start to end...
    if isAntisym
        seq1 = sequence(imgIdx:end);
        silh1 = silhouettes(imgIdx:end);
        seq2 = flip(sequence(1:imgIdx));
        silh2 = flip(silhouettes(1:imgIdx));
    else % or as a circular list of images
        numImgsForSeq = uint8(numImgs/2)-1;

        if imgIdx + numImgsForSeq >= numImgs
            leftIdx = mod((imgIdx + imgIdx),numImgs);
            seq1 = [sequence(imgIdx:end);sequence(1:leftIdx)];
            silh1 = [silhouettes(imgIdx:end);silhouettes(1:leftIdx)];
            seq2 = flip(sequence(leftIdx+1:imgIdx));
            silh2 = flip(silhouettes(leftIdx+1:imgIdx));
        else
            leftIdx = imgIdx + numImgsForSeq;
            seq1 = sequence(imgIdx:leftIdx);
            silh1 = silhouettes(imgIdx:leftIdx);
            seq2 = flip([sequence(leftIdx+1:end);sequence(1:imgIdx)]);
            silh2 = flip([silhouettes(leftIdx+1:end);silhouettes(1:imgIdx)]);
        end
    end
end

function index = detectImageIndex(startImg,sequence)
%
%  Input(s): 
%           startImg - image that breaks the sequence
%           sequence - complete set of images
%  Output(s): 
%           index - index in the sequence of the image that should break it
%

    % Use the Eearth Mover Distance metric to detect the image in the
    % sequence
    startHistR = imhist(startImg(:,:,1));
    startHistG = imhist(startImg(:,:,2));
    startHistB = imhist(startImg(:,:,3));
    
    numImgs = length(sequence);
    distances = zeros(numImgs,3);
    for i = 1:numImgs
        currImg = sequence{i};
        
        % Compute color histograms for the current image and check them
        % against the reference one
        histR = imhist(currImg(:,:,1));
        histG = imhist(currImg(:,:,2));
        histB = imhist(currImg(:,:,3));
        
        distR = emd(startHistR,histR);
        distG = emd(startHistG,histG);
        distB = emd(startHistB,histB);
        
        % Save all the colorimetric distances
        distances(i,:) = [distR,distG,distB];
    end
    
    distMetric = zeros(numImgs,1);
    for i = 1:numImgs
        distMetric(i) = sum(distances(i,:));
    end
    
    index = find(distMetric == min(distMetric));
end