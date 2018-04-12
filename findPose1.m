function vSet = findPose1(mainImg,imgs,cameraParams)

    vSet = poseEstimation(mainImg,imgs,cameraParams);
 
end

function vSet = poseEstimation(firstImg,sideImgs,cameraParams)
    MATCH_THRESH = 10;
    MAX_RATIO = 0.6;
    
    vSet = viewSet;

    firstImg = undistortImage(rgb2gray(firstImg),cameraParams);
    startPoints = detectSURFFeatures(firstImg,'MetricThreshold',1000,'NumOctaves',6,'NumScaleLevels',12);
    [firstFeatures,validFirstPts] = extractFeatures(firstImg,startPoints);
    vSet = addView(vSet, 1, 'Points', startPoints, 'Orientation', eye(3)*rotx(10),...
        'Location',[0 0 0]);
    
    viewId = 2;
    currImg = undistortImage(rgb2gray(sideImgs{1}),cameraParams);
    currPoints = detectSURFFeatures(currImg,'MetricThreshold',1000,'NumOctaves',6,'NumScaleLevels',12);
    [currFeatures,validCurrPts] = extractFeatures(currImg,currPoints);
    
    indexPairs = matchFeatures(firstFeatures,currFeatures,'MatchThreshold',MATCH_THRESH,...
        'MaxRatio',MAX_RATIO,'Unique',true);
    
    matchingStartPts = validFirstPts(indexPairs(:,1),:);
    matchingCurrPts = validCurrPts(indexPairs(:,2),:);

 
    
    [E, inlierIdx] = estimateEssentialMatrix(matchingStartPts,matchingCurrPts,cameraParams,...
        'Confidence',99.99,'MaxDistance',5,'MaxNumTrials',500);
    inlierPointsFirst = matchingStartPts(inlierIdx); 
    inlierPointsCurr = matchingCurrPts(inlierIdx); 
    
    [orient,loc,~] = relativeCameraPose(E,cameraParams,inlierPointsFirst,inlierPointsCurr);
    [R12,t12] = cameraPoseToExtrinsics(orient,loc);
    camMatrix2 = cameraMatrix(cameraParams, R12, t12); 
    camMatrix1 = cameraMatrix(cameraParams,eye(3),[0 0 0]); 
    
    scaleFactor = computeScaleFactor(firstImg,currImg,...
                                     inlierPointsFirst,inlierPointsCurr,...
                                     camMatrix1,camMatrix2);
    loc = loc*scaleFactor;
    
    vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, 'Location', loc); 
    vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);
    
    vSet = estimationCompletion(sideImgs,cameraParams,vSet,currFeatures,validCurrPts); 
end

function vSet = estimationCompletion(imgs,cameraParams,vSet,currFeatures,validCurrFeat)
    numImgs = length(imgs);
    prevFeatures = currFeatures;
    validPrevFeat = validCurrFeat;
    
    for viewId = 3:numImgs+1 % slided forward of 1 image
        currImg = undistortImage(rgb2gray(imgs{viewId-1}),cameraParams);
        currPoints = detectSURFFeatures(currImg,'MetricThreshold',500,'NumOctaves',6,'NumScaleLevels',12);
        [currFeatures,validCurrFeat] = extractFeatures(currImg,currPoints);
        indexPairs = matchFeatures(prevFeatures,currFeatures,'MatchThreshold',20,...
        'MaxRatio',0.7,'Unique',true);
    
        prevImg = undistortImage(rgb2gray(imgs{viewId-2}),cameraParams);
        matchingPtsPrev = validPrevFeat(indexPairs(:,1),:);
        matchingPtsCurr = validCurrFeat(indexPairs(:,2),:);
        showMatchedFeatures(prevImg,currImg,matchingPtsPrev,matchingPtsCurr);

        [worldPointsSURF, imagePointsSURF] = helperFind3Dto2DCorrespondences(vSet,cameraParams,...
            indexPairs,currPoints);

        [orient, loc] = estimateWorldCameraPose(imagePointsSURF, worldPointsSURF,...
            cameraParams,'MaxNumTrials',50000,'Confidence',99,...
            'MaxReprojectionError',0.7);

        vSet = addView(vSet,viewId,'Points',currPoints,'Orientation',orient,...
            'Location',loc);
        vSet = addConnection(vSet,viewId-1,viewId,'Matches',indexPairs);

        if mod(viewId,2)==0
            windowSize = 4;
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
        
        prevFeatures = currFeatures;
        validPrevFeat = validCurrFeat;
    end
end