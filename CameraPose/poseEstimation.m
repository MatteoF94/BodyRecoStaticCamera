function vSet = poseEstimation(imgs,cameraParams)
%POSEESTIMATION Summary of this function goes here
%   INVALID 

    vSet = viewSet;

    viewId = 1;
    prevImg = undistortImage(rgb2gray(imsharpen(imgs{viewId})),cameraParams);
    prevPoints = detectSURFFeatures(prevImg,'MetricThreshold',1000,'NumOctaves',1,'NumScaleLevels',12);
    [prevFeatures,validPrevFeat] = extractFeatures(prevImg,prevPoints);
    vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', eye(3)*rotx(10),...
        'Location',[0 0 0]);
    
    viewId = 2;
    currImg = undistortImage(rgb2gray(imsharpen(imgs{viewId})),cameraParams);
    currPoints = detectSURFFeatures(currImg,'MetricThreshold',1000,'NumOctaves',1,'NumScaleLevels',12);
    [currFeatures,validCurrFeat] = extractFeatures(currImg,currPoints);
    
    indexPairs = matchFeatures(prevFeatures,currFeatures,'MatchThreshold',10,...
        'MaxRatio',0.7,'Unique',true);
    matchingPtsPrev = validPrevFeat(indexPairs(:,1),:);
    matchingPtsCurr = validCurrFeat(indexPairs(:,2),:);
    
    distances = zeros(1,length(matchingPtsPrev));
    for i=1:length(matchingPtsPrev)
        distances(i) = pdist2(matchingPtsPrev(i).Location,matchingPtsCurr(i).Location,'Euclidean');
    end

    matchingPtsPrev = matchingPtsPrev(distances<30);
    matchingPtsCurr = matchingPtsCurr(distances<30);
    %showMatchedFeatures(prevImg,currImg,matchingPtsPrev,matchingPtsCurr);
    
    [E, inlierIdx] = estimateEssentialMatrix(matchingPtsPrev,matchingPtsCurr,cameraParams,...
        'Confidence',99.99,'MaxDistance',10,'MaxNumTrials',500);
    inlierPointsPrev = matchingPtsPrev(inlierIdx); 
    inlierPointsCurr = matchingPtsCurr(inlierIdx); 
    
    [orient,loc,~] = relativeCameraPose(E,cameraParams,inlierPointsPrev,inlierPointsCurr);
    [R12,t12] = cameraPoseToExtrinsics(orient,loc);
    camMatrix2 = cameraMatrix(cameraParams, R12, t12); 
    camMatrix1 = cameraMatrix(cameraParams,eye(3),[0 0 0]); 
    
    scaleFactor = computeScaleFactor(prevImg,currImg,...
                                     inlierPointsPrev,inlierPointsCurr,...
                                     camMatrix1,camMatrix2);
    loc = loc*scaleFactor;


    vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, 'Location', loc); 
    vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);

    vSet = estimationCompletion(imgs,cameraParams,vSet,currFeatures,validCurrFeat); 
end

function vSet = estimationCompletion(imgs,cameraParams,vSet,currFeatures,validCurrFeat)
    numImgs = length(imgs);
    prevFeaturesSURF = currFeatures;
    validPrevFeatSURF = validCurrFeat;
    prevFeaturesFREAK = extractFeatures(undistortImage(rgb2gray(imsharpen(imgs{2})),cameraParams),...
        detectBRISKFeatures(undistortImage(rgb2gray(imsharpen(imgs{2})),cameraParams),'MinContrast',0.01));
    
    for viewId = 3:numImgs/3
        currImg = undistortImage(rgb2gray(imsharpen(imgs{viewId})),cameraParams);
        currPointsSURF = detectSURFFeatures(currImg,'MetricThreshold',500,'NumOctaves',1,'NumScaleLevels',12);
        [currFeaturesSURF,validCurrFeatSURF] = extractFeatures(currImg,currPointsSURF);
        indexPairsSURF = matchFeatures(prevFeaturesSURF,currFeaturesSURF,'MatchThreshold',10,...
        'MaxRatio',0.7,'Unique',true);
    
        currPointsBRISK = detectBRISKFeatures(currImg,'MinContrast',0.01);
        [currFeaturesFREAK,validCurrFeatBRISK] = extractFeatures(currImg,currPointsBRISK);
        indexPairsBRISK = matchFeatures(prevFeaturesFREAK,...
            currFeaturesFREAK,'MatchThreshold',40,'MaxRatio',0.8);
    
        prevImg = undistortImage(rgb2gray(imsharpen(imgs{viewId-1})),cameraParams);
        matchingPtsPrev = validPrevFeatSURF(indexPairsSURF(:,1),:);
        matchingPtsCurr = validCurrFeatSURF(indexPairsSURF(:,2),:);
        showMatchedFeatures(prevImg,currImg,matchingPtsPrev,matchingPtsCurr);

        [worldPointsSURF, imagePointsSURF] = helperFind3Dto2DCorrespondences(vSet,cameraParams,...
            indexPairsSURF,currPointsSURF);
        [worldPointsBRISK, imagePointsBRISK] = helperFind3Dto2DCorrespondences(vSet,cameraParams,...
            indexPairsBRISK,currPointsBRISK);

        [orient, loc] = estimateWorldCameraPose([imagePointsSURF;imagePointsBRISK],...
            [worldPointsSURF;worldPointsBRISK],...
            cameraParams,'MaxNumTrials',100000,'Confidence',99,...
            'MaxReprojectionError',1);

        vSet = addView(vSet,viewId,'Points',currPointsSURF,'Orientation',orient,...
            'Location',loc);
        vSet = addConnection(vSet,viewId-1,viewId,'Matches',indexPairsSURF);

        if mod(viewId,4) == 1
            windowSize = 3;
            startFrame = max(1,viewId - windowSize);
            tracks = findTracks(vSet, startFrame:viewId);
            camPoses = poses(vSet, startFrame:viewId);
            [xyzPoints, reprojErrors] = triangulateMultiview(tracks, camPoses, ...
                cameraParams);

            fixedIds = [startFrame,startFrame+1];
            idx = reprojErrors < 2;
            [~,camPoses] = bundleAdjustment(xyzPoints(idx,:),tracks(idx), ...
                camPoses, cameraParams, 'FixedViewIDs', fixedIds, ...
                'PointsUndistorted', true, 'AbsoluteTolerance', 1e-9, ...
                'RelativeTolerance', 1e-9, 'MaxIterations', 500);

            vSet = updateView(vSet, camPoses);
        end
        
        prevFeaturesSURF = currFeatures;
        validPrevFeatSURF = validCurrFeat;
        prevFeaturesFREAK = currFeaturesFREAK;
    end
end

function scale = computeScaleFactor(imgP,imgC,inlierP,inlierC,camP,camC)
    figure, imshow(imgP), hold on, plot(inlierP);
    [x1,y1] = getpts();
    close all;
    figure, imshow(imgC), hold on, plot(inlierC);
    [x2,y2] = getpts();
    close all;
    
    distances1 = zeros(2,length(inlierP));
    distances2 = zeros(2,length(inlierC));
    
    for i = 1:length(inlierP)
        ip1 = inlierP(i).Location;
        ip2 = inlierC(i).Location;
        distances1(1,i) = sqrt((x1(1)-ip1(1))^2+((y1(1)-ip1(2))^2));
        distances1(2,i) = sqrt((x1(2)-ip1(1))^2+((y1(2)-ip1(2))^2));
        distances2(1,i) = sqrt((x2(1)-ip2(1))^2+((y2(1)-ip2(2))^2));
        distances2(2,i) = sqrt((x2(2)-ip2(1))^2+((y2(2)-ip2(2))^2));
    end

    ptOld1 = inlierP(distances1(1,:)==min(distances1(1,:))).Location;
    ptOld2 = inlierP(distances1(2,:)==min(distances1(2,:))).Location;
    ptNew1 = inlierC(distances2(1,:)==min(distances2(1,:))).Location;
    ptNew2 = inlierC(distances2(2,:)==min(distances2(2,:))).Location;

    knownObj = triangulate([ptOld1', ptOld2'],[ptNew1', ptNew2'],camP,camC);
    scale = double(0.4/norm(knownObj(1,:)-knownObj(2,:)));
end
