function silhouettes = bkgMonochromSubtraction(bkgs,imgs)
%
% (NOT USED IN THE PROJECT)
%
%  Input(s):
%           bkgs - sequence of background images
%           imgs - set of images containing the moving target
%  Output(s):
%           silhouettes - extracted foregrounds from the input images
%

    % Model the background as a simple mean
    numBkgs = length(bkgs);
    bkg = zeros(size(bkgs{1}),'uint32');
    for i = 1:numBkgs
        bkg = bkg + uint32(bkgs{i});
    end
    bkg = uint8(bkg/numBkgs);
    
    rB = bkg(:,:,1);
    gB = bkg(:,:,2);
    bB = bkg(:,:,3);
    
    % Compute each colorimetric threshold as sum of ratios between channels
    rgRatio = rB./gB;
    bgRatio = bB./gB;
    rgThresh = 0.5*double((max(max(rgRatio)) + min(min(rgRatio))));
    bgThresh = 0.5*double((max(max(bgRatio)) + min(min(bgRatio))));
    
    % Extract the foregrounds applying channels ratio thresholding
    numImgs = length(imgs);
    silhouettes = cell(numImgs,1);
    for i = 1:numImgs
        currImg = imgs{i};
        rC = currImg(:,:,1);
        gC = currImg(:,:,2);
        bC = currImg(:,:,3);
        silhouettes{i} = ~(((rC./gC) <= rgThresh) & (bC./gC <= bgThresh));
    end
end