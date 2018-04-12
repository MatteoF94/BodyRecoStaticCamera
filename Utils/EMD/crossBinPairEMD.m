function distanceRGB = crossBinPairEMD(imgA,imgB)
%CROSSBINPAIREMD Summary of this function goes here
%   Detailed explanation goes here
    histRedA = imhist(imgA(:,:,1));
    histGreenA = imhist(imgA(:,:,2));
    histBlueA = imhist(imgA(:,:,3));
    
    histRedB = imhist(imgB(:,:,1));
    histGreenB = imhist(imgB(:,:,2));
    histBlueB = imhist(imgB(:,:,3));
    
    distR = emd(histRedA,histRedB);
    distG = emd(histGreenA,histGreenB);
    distB = emd(histBlueA,histBlueB);

    distanceRGB = [distR,distG,distB];
end

