function silhouettes = hsvSubtraction(bkgs,imgs,varargin)
    % Input arguments error checks and parsing
    if length(bkgs) < 1
        error('myfuns:hsvSubtraction:TooFewInputs', ...
            'Function requires at least one background image.');
    end
    
    if length(varargin) < 1
        filtWinSize = 3; % default filter window size
    else
        filtWinSize = varargin{1};
    end
    
    % To model the background we need to extract separaterly the three
    % color channel for all the available background images and compute the
    % average
    numBkgs = length(bkgs);
    bkg = zeros(size(bkgs{1}),'uint32');
    for i = 1:numBkgs
        bkg = bkg + uint32(bkgs{i});
    end
    bkg = uint8(bkg/numBkgs);
    bkgHSV = round(rgb2hsv(bkg)); % convert background in hsv color space
    
    % Each image is compared in the HSV color space with the background,
    % using a bitwise xor comparison (positive output if two pixels
    % differ). Application of median filtering and region denoising
    % provides a better outcome, coping with the not considered variance in
    % the background
    numImgs = length(imgs);
    silhouettes = cell(numImgs,1);
    
    for i = 1:numImgs
        imgHSV = round(rgb2hsv(imgs{i}));
        out = rgb2gray(bitxor(bkgHSV,imgHSV));
        binImg = out>0;
        
        filtImg = medfilt2(binImg,[filtWinSize,filtWinSize]);
        
        % Denoising done clustering similar regions in the binary image
        [L, num] = bwlabel(filtImg);
        STATS = regionprops(L,'all');
        for j = 1:num
            dd=STATS(i).Area;
            if(dd<500)
                L(L==j)=0;
                num=num-1;
            end
        end

        [L2,~] = bwlabel(L);
        silhouettes{i} = L2;
    end
end