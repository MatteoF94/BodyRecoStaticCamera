function silhouettes = foregroundTM(input_imgs,w,st,k)
%
%  Finds background pixels.
%  Inputs: input_imgs - images in which we want to extract silhouettes
%          w - the background model
%          st - the noise estimate for each pixel (image same size as w)
%          k - tolerance for foreground detection (scalar)
%
%  Output: back - stack of binary images with the final foreground results
%
    numImgs = length(input_imgs);

    mask_std =[ 0 0 1 0 0; 0 1 1 1 0;1 1 144 1 1;0 1 1 1 0;0 0 1 0 0]/13^2;
    std2 = wkeep(conv2(mask_std,(st./(w+.0001)).^2),size(w),'c');
    mask = [1 2 1;2 4 2;1 2 1]; 
    smask = sum(mask(:));
    
    if length(st)>1
        st2 = conv2(st+1.01/smask,mask,'same');
    else
        st2 = smask*st;
    end
    
    silhouettes = cell(numImgs,1);
    for i = 1:numImgs
        y = input_imgs{i};
        
        if length(size(y))>2
            x = rgb2gray(y);
        else
            x = y;
        end

        bw = conv2(abs(double(x)-double(w)),mask,'same');  %faz o somatorio em torno de cada pixel para um quadro
        d1 = abs(bw)>k*st2;

        el1 = strel('diamond',2);
        el2 = strel('diamond',3);
        d1 = imclose(imopen(d1,el1),el2); % initial foreground pixels
        [ba,~,~] = detect_shadow_highlight_gaussian(x,w,d1,std2,.7,1);
      
        d3 = imclose(imopen(d1&not(ba),el1),el2); % foreground pixels after removing shadows and highlights
    
        silhouettes{i}  =  d3;
    end
   
end

