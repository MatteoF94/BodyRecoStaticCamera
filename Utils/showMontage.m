function showMontage(imgs)
    montageImgs = cat(4,imgs{:}); 
    
    montage(montageImgs);
end