function maskedImgs = detachForeground(imgs,silhouettes)
%
%  Input(s):
%           imgs - sequence of images representing the moving target
%           silhouettes - the extracted silhouettes from the input images
%  Output(s):
%           maskedImgs - extracted foregrounds from the images
%

    % Input arguments error checks 
    if length(imgs) ~= length(silhouettes)
        error('myfuns:extractForeground:DifferentInputs', ...
            'Function requires as many silhouettes as the scene images.');
    end
    
    numElems = length(imgs);
    maskedImgs = cell(numElems,1);
    for i = 1:numElems
        maskedRgbImage = bsxfun(@times, imgs{i},...
                                cast(silhouettes{i},class(imgs{i})));
        maskedImgs{i} = maskedRgbImage;
    end
end

