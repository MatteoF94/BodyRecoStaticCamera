%% Image loading and preparation
addpath(genpath('../trunk'));

input_bkgs = loadImages('Dataset/Demo2/bkg','jpg',0);
input_seq = loadImages('Dataset/Demo2/seq','jpg',1);
%%

% Silhouettes extraction exploiting the HSV color space. The filter size is
% optional, 3 as default. For this demo, a 5x5 window provides better
% results
silhHSVSub = hsvSubtraction(input_bkgs, input_seq,2);
%%
% Silhouettes extraction using the subtraction technique described in
% "Efficient Background Subtraction and Shadow Removal for Monochromatic 
% Video Sequences. IEEE Transactions on Multimedia".
[w,~,k,st] = backgroundTM(input_bkgs);
silhEBSSR = foregroundTM(input_seq,w,st,k);
for i = 1:length(silhEBSSR)
    silhEBSSR{i} = medfilt2(silhEBSSR{i},[7,7]);
end
showMontage(silhEBSSR);

%%
% Silhouettes extraction using the subtraction technique described in 
% "Adaptive Background Mixture Models for Real-Time Tracking" and 
% "An Improved Adaptive Background Mixture Model for Realtime Tracking with
% Shadow Detection", both implemented in Matlab in the vision toolbox
silhGMM = gaussianMixtureModelSubtraction(input_bkgs,input_seq,...
    struct('param','eta','value',0.0000001),...
    struct('param','numGaussians','value',15),...
    struct('param','bkgRatio','value',0.7),...
    struct('param','var','value','Auto'));

for i = 1:length(silhGMM)
    %miao{i} = imdilate(imerode(silhGMM{i},strel('disk',3,0)),strel('disk',7,0));
    silhGMM{i} = medfilt2(silhGMM{i},[9,9]);
end
showMontage(silhGMM);

%%
% Compute the single channel thresholds using genetic algorithm
[rT,gT,bT] = geneticGraySelection(input_bkgs,input_seq{1},...
    silhGMM{1},20,100);

% Silhouettes extraction exploiting separate thresholds for each channel of
% the RGB color space. The thresholds are optional, 4 as default each. For
% this demo, the previous genetic algorithm provides the best combination
% of them: 14 (red), 17 (green), 10 (blue)
silhStdSub = grayscaleSubtraction(input_bkgs,input_seq,...
    struct('ch','r','value',rT),...
    struct('ch','g','value',gT),...
    struct('ch','b','value',bT));

%% Silhouette selection and preparation
% Save the images representing the silhouettes in a directory, for later
% usage
silhPath = strcat(pwd,'/Dataset/Demo2/silhs');
if ~exist(silhPath,'dir')
    mkdir(silhPath);
    addpath(silhPath); % if it exist it has been already inserted in the Matlab datapath
end

for i = 1:length(silhGMM)
    imwrite(silhGMM{i},strcat(silhPath,'/',sprintf('SILH_%.4d.jpg',i)));
end

%% Foreground extraction (using gaussian mixture model)
%foreGMM = cell(length(input_seq),1);
%for i = 1:length(input_seq)
    %foreGMM{i} = bsxfun(@times, input_seq{i}, cast(silhGMM{i},class(input_seq{i})));
%end
foreGMM = detachForeground(input_seq,silhGMM);

% Save the images representing the foreground in a directory, for later
% usage
fgPath = strcat(pwd,'/Dataset/Demo2/fg');
if ~exist(fgPath,'dir')
    mkdir(fgPath);
    addpath(fgPath); % if it exist it has been already inserted in the Matlab datapath
end

for i = 1:length(foreGMM)
    imwrite(foreGMM{i},strcat(fgPath,'/',sprintf('FG_%.4d.jpg',i)));
end

%% Pose estimation preparation
% First we need to order the image, according to the rotation. 
[ordFg,ordSilh] = imageOrdering(fgPath,silhPath);
%%
% Then we remove the duplicate images, since they do not bring new
% information and just slow down the computational time. An acceptable
% sensitivity for the demo is around 5 pixels. 
[cleanFg,cleanSilh] = wipeDuplicates(ordFg,ordSilh,5);
%%
% Lastly, we select the images that are suited for the pose estimation
% procedure. If an image in the sequence has not a sufficient number of
% matching features with the previous one (the sequence is ORDERED), then 
% the corresponding camera position cannot be computed
[montagin,mask] = selectionByFeatures(cleanFg,120);
joinedImgs = findLargestImgSubset(cleanFg,mask);
joinedSilhs = findLargestImgSubset(cleanSilh,mask);

%% Relative pose estimation
% Before proceeding we neew to break the obtained sequence of images in two
% senses, given a starting image. This is selected manually* and the two
% subsequences are easily found exploiting the fact that the sequence is
% ordered with respect to the rotation of the target. 
refImg = joinedImgs{1};
[imgsL,imgsR,silhsL,silhsR] = breakSequence(refImg,joinedImgs,joinedSilhs);

%%
% Since we are dealing with static camera and moving subject, to find
% ourselves in the state of the art we can imagine once again a rotating
% camera, but without considering the background (making the two cases
% undistinguishable)
cameraParams = calibrationDemo2();

%%
vSetL = findPosesKLT(refImg,imgsL,cameraParams);
vSetR = findPosesKLT(refImg,imgsR,cameraParams);

%% Voxel carving
% Initialization of the voxels, giving an imprecise guess over the target
% position.
delta = 0.005;
voxel_size = [delta delta delta];
xlim = [-0.3 0.3]; 
ylim = [-0.5 0.5]; 
zlim = [1.2 1.7];
[voxels,voxel3Dx,voxel3Dy,voxel3Dz,voxels_number] = initializeVoxels(xlim,ylim,zlim,voxel_size);

voxels = carvingFromPoses(voxels,vSetL,vSetR,...
    [joinedSilhs(1);silhsL],...
    [joinedSilhs(1);silhsR],cameraParams);