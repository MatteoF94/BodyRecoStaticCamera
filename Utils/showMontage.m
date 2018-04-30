function showMontage(imgs)
%
%  Input(s):
%           imgs - sequence of images to be showed as a montage
%

    montageImgs = cat(4,imgs{:}); 
    montage(montageImgs);
end