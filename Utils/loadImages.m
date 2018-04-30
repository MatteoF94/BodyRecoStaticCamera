function images = loadImages(folderName, format, montageOn)
%
%  Input(s):
%           folderName - name of the folder containing the images
%           format - format of the images to be loaded
%           montageOn - 1 if we want to show a montage of all the loaded
%                       images, 0 otherwise
%  Output(s):
%           images - loaded images
%
    imgRoot = strcat('./',folderName,'/');
    imnames=dir([imgRoot '*' format]);
    
    num_imgs = length(imnames); 
    images = cell(num_imgs,1);

    for i=1:num_imgs   
        imname=[imgRoot imnames(i).name]; 
        images{i} = imread(imname);
    end
    
    if montageOn
        folderPaths = {imnames.folder}';
        names = {imnames.name}';
        montage(strcat(folderPaths,'/',names));
    end
end