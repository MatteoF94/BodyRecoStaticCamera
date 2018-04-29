%% ----------------------------------------------------------------------%%
% ----------------------- SEQUENCES PREPARATION --------------------------%
% ------------------------------------------------------------------------%
addpath(genpath('../trunk'));

input_bkgs = loadImages('Dataset/Demo1/bkg','jpg',0);
input_seq = loadImages('Dataset/Demo1/seq','jpg',1);


%% ----------------------------------------------------------------------%%
% ----------------------- SILHOUETTES EXTRACTION -------------------------%
% ------------------------------------------------------------------------%
%% HSV background subtraction
% Silhouettes extraction exploiting the HSV color space. The filter size is
% optional, 3 as default. For this demo, a 5x5 window provides better
% results.
silhHSVSub = hsvSubtraction(input_bkgs, input_seq,5);
showMontage(silhHSVSub);
title('HSV silhouette extraction')

%% Efficient background subtraction 
% Silhouettes extraction using the subtraction technique described in
% "Efficient Background Subtraction and Shadow Removal for Monochromatic 
% Video Sequences. IEEE Transactions on Multimedia".
[w,~,k,st] = backgroundTM(input_bkgs);
silhEBSSR = foregroundTM(input_seq,w,st,k);
showMontage(silhEBSSR);
title('EBBSR silhouette extraction')

%% Adaptive background mixture model (gaussian background subtraction)
% Silhouettes extraction using the subtraction technique described in 
% "Adaptive Background Mixture Models for Real-Time Tracking" and 
% "An Improved Adaptive Background Mixture Model for Realtime Tracking with
% Shadow Detection", both implemented in Matlab in the vision toolbox.
close all;
silhGMM = gaussianMixtureModelSubtraction(input_bkgs,input_seq,...
    struct('param','eta','value',0.0000001),...
    struct('param','numGaussians','value',3),...
    struct('param','bkgRatio','value',0.7),...
    struct('param','var','value','Auto'));
showMontage(silhGMM);
title('Adaptive background mixture model (gaussian background modeling)')

%% Statistical background modeling and classification
% Silhouettes extraction using the subtraction technique described in 
% "A statistical approach for real-time robust background subtraction and 
% shadow detection".
[mean_, stddev_, brightness_,bdist_variation,cdist_variation,...
    thresh_cdist,thresh_bdist_left,thresh_bdist_right] = statisticalBackgroundModeling(input_bkgs(1:15),0.99);
[silhStatist,~,~] = statisticalClassification(input_seq,bdist_variation,cdist_variation,thresh_cdist,thresh_bdist_left,thresh_bdist_right,mean_,stddev_,brightness_);

for i = 1:length(silhStatist)
    silhStatist{i} = medfilt2(silhStatist{i},[15,15]);
end
showMontage(silhStatist);
title('Statistical background modeling and classification')

%% RGB thresholding with Genetic Algorithm
% Compute the single channel thresholds using a genetic algorithm, using as
% ground truth one silhouette extracted with the adaptive mixture model.
[rT,gT,bT] = geneticGraySelection(input_bkgs,input_seq{1},silhGMM{1},20,100);

% Silhouettes extraction exploiting separate thresholds for each channel of
% the RGB color space. The thresholds are optional, 4 as default each.
silhStdSub = grayscaleSubtraction(input_bkgs,input_seq,...
    struct('ch','r','value',rT),...
    struct('ch','g','value',gT),...
    struct('ch','b','value',bT));
showMontage(silhStdSub);
title('RGB thresholding with Genetic algorithm')


%% ----------------------------------------------------------------------%%
% ------------------------ FOREGROUND EXTRACTION -------------------------%
% ------------------------------------------------------------------------%
%% Silhouettes preparation
% Save the images representing the silhouettes in a directory, for later
% usage.
silhPath = strcat(pwd,'/Dataset/Demo1/silhs');
if ~exist(silhPath,'dir')
    mkdir(silhPath);
    addpath(silhPath); % if it exist it has been already inserted in the Matlab datapath
else
    delete(strcat(silhPath,'/*'));
end

for i = 1:length(silhGMM)
    imwrite(silhGMM{i},strcat(silhPath,'/',sprintf('SILH_%.4d.jpg',i)));
end

%% Foreground extraction (silhouettes from gaussian mixture model)
% Find the foreground in each image of the input sequence combining images
% and silhouettes.
foreGMM = detachForeground(input_seq,silhGMM);

% Save the images representing the foreground in a directory, for later
% usage.
fgPath = strcat(pwd,'/Dataset/Demo1/fg');
if ~exist(fgPath,'dir')
    mkdir(fgPath);
    addpath(fgPath); % if it exist it has been already inserted in the Matlab datapath
else
    delete(strcat(fgPath,'/*'));
end

for i = 1:length(foreGMM)
    imwrite(foreGMM{i},strcat(fgPath,'/',sprintf('FG_%.4d.jpg',i)));
end


%% ----------------------------------------------------------------------%%
% -------------------- CAMERA/TARGET POSE ESTIMATION ---------------------%
% ------------------------------------------------------------------------%
% First we need to order the image, according to the rotation. 
[ordFg,ordSilh] = imageOrdering(fgPath,silhPath);
clear silhPath fgPath

% Then we remove the duplicate images, since they do not bring new
% information and just slow down the computational time. An acceptable
% sensitivity for the demo is around 5 pixels. 
[cleanFg,cleanSilh] = wipeDuplicates(ordFg,ordSilh,5);

% Then we select the images that are suited for the pose estimation
% procedure. If an image in the sequence has not a sufficient number of
% matching features with the previous one (the sequence is ORDERED), then 
% the corresponding camera position cannot be computed.
[~,mask] = selectionByFeatures(cleanFg,165);
joinedImgs = findLargestImgSubset(cleanFg,mask);
joinedSilhs = findLargestImgSubset(cleanSilh,mask);

% Lastly, we need to break the obtained sequence of images in two senses,
% given a starting image. This is selected manually and the two
% subsequences are easily found exploiting the fact that the sequence is
% ordered with respect to the rotation of the target.  
refImg = joinedImgs{9};
[imgsL,imgsR,silhsL,silhsR] = breakSequence(refImg,joinedImgs,joinedSilhs,1);

%% Camera calibration
% Using images of a checkboard with known square size, we calibrate the
% camera and find the intrinsic parameters.
cameraParams = calibrationDemo1();

%% Pose estimation from simple matching
vSetMatchingL = findPoseMatching(imgsL,cameraParams,0.4);
vSetMatchingR = findPoseMatching(imgsR,cameraParams,0.4);

%% Pose estimation from interrupted KLT
vSetKltL = findPosesKLT(imgsL,cameraParams,0.4);
vSetKltR = findPosesKLT(imgsR,cameraParams,0.4);


%% ----------------------------------------------------------------------%%
% ---------------------- VOLUMETRIC RECONSTRUCTION -----------------------%
% ------------------------------------------------------------------------%
%% Voxel carving
% Initialization of the voxels, giving an imprecise guess over the target
% position. 
[xlim,ylim,zlim] = findModelBoundaries(vSetMatchingL,vSetMatchingR,silhsL,silhsR,0,0,cameraParams);
%%
% Distribute the rough limits in more voxels, to obtain a more precise
% model. A dense reconstruction can be achieved with a discretisation of
% 20M voxels. 
delta = nthroot((xlim(2)-xlim(1))*(ylim(2)-ylim(1))*(zlim(2)-zlim(1))/20000000,3);
voxelSize = [delta delta delta];
[voxels,~,~,~,~] = initializeVoxels(xlim,ylim,zlim,voxelSize);

% Carve the model from the discretization of the bounded volume.
voxels = carvingFromPoses(voxels,vSetMatchingL,vSetMatchingR,...
    silhsL,silhsR,0,0,cameraParams);

%% Voxel visualisation
% Visualize the target with the MATLAB built in 3D plotting function.
figure, pcshow(voxels(voxels(:,4)>=vSetMatchingL.NumViews+vSetMatchingR.NumViews,1:3));
view([0,-80])
colormap summer

% Visualize the target using the computed voxels and not a dense cloud of
% points.
figure, voxelPatch = create3DReconstruction(voxels(voxels(:,4)>=...
    vSetMatchingL.NumViews+vSetMatchingR.NumViews,:),delta);