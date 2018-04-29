function [mean_, stddev_, brightness_,bdist_variation, cdist_variation,thresh_cdist,thresh_bdist_left,thresh_bdist_right] = statisticalBackgroundModeling(imgs, det_rate)
%
%  Input(s):
%           imgs - images of the background, used to average over time
%           det_rate - detection rate of pixels
%  Output(s):
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
%   

    [mean_, stddev_, brightness_] = modelMeanStdDev();
    [bdist_variation, cdist_variation] = computeVariations();
    [bdist_norm, cdist_norm] = computeNormalizedDistortions();
    [thresh_cdist,thresh_bdist_left,thresh_bdist_right] = selectThreshold();

    function [mean_, stddev_, brightness_] = modelMeanStdDev()
        mean_ = zeros(size(imgs{1}));
        stddev_ = zeros(size(imgs{1}));
        brightness_ = zeros(size(imgs{1}));
        pixels = zeros(length(imgs),3);
    
        rows = size(imgs{1},1);
        columns = size(imgs{1},2);
    
        for y = 1:rows
            for x = 1:columns
                for id_image = 1:length(imgs)
                    pixels(id_image,:) = imgs{id_image}(y,x,:);
                end
            
                mean_pix = zeros(1,3);
                stddev_pix = zeros(1,3);
                for ch = 1:3
                    mean_pix(ch) = mean(pixels(:,ch));
                    stddev_pix(ch) = std(pixels(:,ch));
                end
            
                for id = 1:3
                    if stddev_pix(id) == 0
                        stddev_pix(id) = 1;
                    end
                end
            
                denom = computeBrightnessDenominator(mean_pix,stddev_pix);
            
                for id = 1:3
                    brightness_(y,x,id) = mean_pix(id)/(denom*stddev_pix(id)*stddev_pix(id));
                    mean_(y,x,id) = mean_pix(id);
                    stddev_(y,x,id) = stddev_pix(id);
                end
            end
        end
    end

    function denom = computeBrightnessDenominator(mean_p,stddev_p)
        r = mean_p(1)/stddev_p(1);
        g = mean_p(2)/stddev_p(2);
        b = mean_p(3)/stddev_p(3);
        denom = b*b + g*g + r*r;
        if denom == 0
            denom = 1;
        end
    end

    function [bdist_variation, cdist_variation] = computeVariations()
        bdist_variation = zeros(size(imgs{1}(:,:,1)));
        cdist_variation = zeros(size(imgs{1}(:,:,1)));
        nb_pixels = length(imgs);
    
        rows = size(imgs{1},1);
        columns = size(imgs{1},2);
    
        for y = 1:rows
            for x = 1:columns
                bdist_sum = 0;
                cdist_sum = 0;
            
                for id_image = 1:nb_pixels
                    bdist_current = computeBrightnessDistortion(imgs{id_image},y,x);
                    cdist_current = computeChromaticityDistortion(imgs{id_image},y,x,bdist_current);
                
                    bdist_sum = bdist_sum + (bdist_current - 1)*(bdist_current - 1);
                    cdist_sum = cdist_current*cdist_current;
                end
            
                bdist_variation(y,x) = sqrt(double(bdist_sum/nb_pixels));
                cdist_variation(y,x) = sqrt(double(cdist_sum/nb_pixels));
                if(cdist_variation(y,x) < 1)
                    cdist_variation(y,x) = 1;
                end
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

    function [out_bdist_norm, out_cdist_norm] = computeNormalizedDistortions()
        rows = size(imgs{1},1);
        cols = size(imgs{1},2);
    
        out_bdist_norm = zeros(rows,cols*length(imgs));
        out_cdist_norm = zeros(rows,cols*length(imgs));
    
        for id_image = 1:length(imgs)
            [out_bdist_norm(:,(id_image-1)*cols+1:id_image*cols),...
                out_cdist_norm(:,(id_image-1)*cols+1:id_image*cols)] = computeBDIST(...
                out_bdist_norm(:,(id_image-1)*cols+1:id_image*cols),...
                out_cdist_norm(:,(id_image-1)*cols+1:id_image*cols),rows,cols,id_image);       
        end
    end

    function [bdist_image,cdist_image] = computeBDIST(bdist_image,cdist_image,rows,cols,id_image)
       for y = 1:rows
           for x = 1:cols
                    bdist_current = computeBrightnessDistortion(imgs{id_image},y,x);
                    cdist_current = computeChromaticityDistortion(imgs{id_image},y,x,bdist_current);
                
                    bdist_image(y,x) = (bdist_current - 1)/bdist_variation(y,x);
                    cdist_image(y,x) = cdist_current/cdist_variation(y,x);
           end
       end
    end

    function [thresh_cdist,thresh_bdist_left,thresh_bdist_right] = selectThreshold()
        [~,thresh_cdist] = selectThresholdMatrix(cdist_norm);
        [thresh_bdist_left,thresh_bdist_right] = selectThresholdMatrix(bdist_norm);
    end

    function [left, right] = selectThresholdMatrix(mat)
        rows = size(mat,1);
        cols = size(mat,2);
        tot_size = rows*cols;
        array = zeros(1,tot_size);
        
        for y = 1:rows
            for x = 1:cols
                array(x+y*cols) = mat(y,x);
            end
        end
        
        index_left = uint32((1 - det_rate)*tot_size);
        index_right = uint32(det_rate*tot_size);
        array = sort(array);
        left = array(index_left);
        right = array(index_right);
    end       
end