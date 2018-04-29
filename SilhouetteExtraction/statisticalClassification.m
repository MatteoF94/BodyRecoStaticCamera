function [silhs,bkgs,shadows] = statisticalClassification(images,bdist_variation,cdist_variation,thresh_cdist,thresh_bdist_left,thresh_bdist_right,mean_,stddev_,brightness_)
%
%  Input(s):
%           images - sequence of images containing the target, to be
%                    classified in foreground and background
%           mean_ - mean of the background images
%           stddev_ - standard deviation of the background images
%           brightness_ - brightness model of the background images
%           bdist_variation - brightness distortion model
%           cdist_variation - chromatic distortion model
%           thresh_cdist - threshold associated to cromatic distortion
%           thresh_bdist_left - threhsold associate to left brightness
%                               distortion
%           thresh_bdist_right - threhsold associate to right brightness
%                                distortion
%  Output(s):
%           silhs - extracted foregrounds as binary masks
%           bkgs - extracted backgrounds as binary masks
%           shadows - extracted shadowed backgrounds as binary masks
%  

    numImgs = length(images);
    silhs = cell(numImgs,1);
    bkgs = cell(numImgs,1);
    shadows = cell(numImgs,1);
    
    for i = 1:numImgs
        [silhs{i},bkgs{i},shadows{i}] = ...
            single_classification(images{i},bdist_variation,cdist_variation,...
            thresh_cdist,thresh_bdist_left,thresh_bdist_right,...
            mean_,stddev_,brightness_);
    end
end

function [mask_foreground, mask_background, mask_shadow] = single_classification(img,bdist_variation,cdist_variation,thresh_cdist,thresh_bdist_left,thresh_bdist_right,mean_,stddev_,brightness_)
    [bdist_norm, cdist_norm] = computeNormalizedDistortion();
    mask_foreground = cdist_norm>thresh_cdist;
    mask_background_left = bdist_norm>thresh_bdist_left;
    mask_background_right = bdist_norm<thresh_bdist_right;
    mask_background = mask_background_left & mask_background_right;
    mask_shadow = -bdist_norm>0;

    function [out_bdist_norm, out_cdist_norm] = computeNormalizedDistortion()
        rows = size(img,1);
        cols = size(img,2);
    
        out_bdist_norm = zeros(rows,cols);
        out_cdist_norm = zeros(rows,cols);
    
        for y = 1:rows
           for x = 1:cols
                    bdist_current = computeBrightnessDistortion(img,y,x);
                    cdist_current = computeChromaticityDistortion(img,y,x,bdist_current);
                
                    out_bdist_norm(y,x) = (bdist_current - 1)/bdist_variation(y,x);
                    out_cdist_norm(y,x) = cdist_current/cdist_variation(y,x);
           end
       end
    end

    function bdist_current = computeBrightnessDistortion(img,y,x)
        brightness = squeeze(brightness_(y,x,:));
        pixel = squeeze(img(y,x,:));
        bdist_current = 0;
    
        for id = 1:3
            bdist_current = bdist_current + pixel(id)*brightness(id);
        end
    end

    function cdist_current = computeChromaticityDistortion(img,y,x,bdist)
        mean_p = squeeze(mean_(y,x,:));
        stddev_p = squeeze(stddev_(y,x,:));
        pixel = squeeze(img(y,x,:));
    
        r = (pixel(1) - bdist*mean_p(1))/stddev_p(1);
        g = (pixel(2) - bdist*mean_p(2))/stddev_p(2);
        b = (pixel(3) - bdist*mean_p(3))/stddev_p(3);
    
        cdist_current = r*r + g*g + b*b;
    end
end