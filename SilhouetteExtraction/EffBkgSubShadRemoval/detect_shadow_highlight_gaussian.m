function [yall,ys,yh]=detect_shadow_highlight_gaussian(x,w,bw,std2,Tlow,Thigh)
%
%  Detect shadows abd highlights for frame x, based on the background model
%  w, detected foreground pixels bw (binatry image), an estimate of the
%  standard deviation of pixel ratios st2, and thresholds Tlow and Thigh
%  used to detect respectively shadows and highlights 
%
%  Output: ys - pixels marked as shadows
%          yh - pixels marked as highlights
%          yall - union of ys and yh 
%
    k_shad=3;
    mask=[ 0 0 1 0 0; 0 1 1 1 0;1 1 1 1 1;0 1 1 1 0;0 0 1 0 0]/13;
    x=double(x)+1e-5;w=double(w)+1e-5;
  
    % Pixel rations
    r=x./w;

    % Computing the mean
    media=ordfilt2(r,7,13*mask);
    
    % Applying the thresholds
    shad=((r-media).^2<k_shad^2*std2);
    y=shad&ordfilt2(shad,7,round(13*mask))&bw;
    ys=y&(media>Tlow)&(media<1);
    yh=y&(media<Thigh)&(media>1);
    fore=bw&not(ys);
    forehole=bwfill(fore,'holes');
    ys(ys&forehole)=0;
    yall=ys|yh;
end