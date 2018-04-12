function vSet = findPosesKLT(mainImg,imgs,cameraParams)
    vSet = viewSet;
    tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

    viewId = 1;
    prevImg = undistortImage(rgb2gray(mainImg),cameraParams);
    prevPoints = detectSURFFeatures(prevImg,'MetricThreshold',500,'NumOctaves',6,'NumScaleLevels',12);
    prevPoints = prevPoints.Location;
    initialize(tracker,prevPoints,prevImg);
    vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', eye(3),...
        'Location',[0 0 0]);

    viewId = 2;
    currImg = undistortImage(rgb2gray(imgs{viewId-1}),cameraParams);
    currPointsSURF = detectSURFFeatures(currImg,'MetricThreshold',3000,'NumOctaves',6,'NumScaleLevels',12);
    [currPoints, validIdx] = step(tracker, currImg);
    
    matchingPtsPrev = prevPoints(validIdx,:);
    matchingPtsCurr = currPoints(validIdx,:);

    %figure, showMatchedFeatures(prevImg,currImg,matchingPtsPrev,matchingPtsCurr);
    %title('Tracked features');

    [E, inlierIdx] = estimateEssentialMatrix(matchingPtsPrev,matchingPtsCurr,cameraParams,...
        'Confidence',99.99,'MaxDistance',5,'MaxNumTrials',3000);
    inlierPointsPrev = matchingPtsPrev(inlierIdx,:); 
    inlierPointsCurr = matchingPtsCurr(inlierIdx,:); 
    %figure; showMatchedFeatures(prevImg,currImg,inlierPointsPrev,inlierPointsCurr);
    %title('Epipolar Inliers');

    [orient,loc,~] = relativeCameraPose(E,cameraParams,inlierPointsPrev,inlierPointsCurr);
    [R12,t12] = cameraPoseToExtrinsics(orient,loc);
    camMatrix2 = cameraMatrix(cameraParams, R12, t12); 
    camMatrix1 = cameraMatrix(cameraParams,eye(3)*rotx(10),[0 0 0]); 
    
    scaleFactor = computeScaleFactor(prevImg,currImg,...
                                     SURFPoints(inlierPointsPrev),SURFPoints(inlierPointsCurr),...
                                     camMatrix1,camMatrix2);
    loc = loc*scaleFactor;
    
    realCurrPoints = [SURFPoints(currPoints);currPointsSURF];
    vSet = addView(vSet, viewId, 'Points', realCurrPoints, 'Orientation', orient, 'Location', loc); 
    vSet = addConnection(vSet, viewId-1, viewId, 'Matches', [find(validIdx),find(validIdx)]);
    
    vSet = estimationCompletion(imgs,cameraParams,vSet,tracker,realCurrPoints); 
end

function vSet = estimationCompletion(imgs,cameraParams,vSet,tracker,points)
    numImgs = length(imgs);
    release(tracker);
    initialize(tracker,points.Location,undistortImage(rgb2gray(imgs{1}),cameraParams));
    f = waitbar(2/(numImgs+1),'1','Name','Camera poses estimation',...
    'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');

    for viewId = 3:numImgs+1 % slided forward of 1 image
        waitbar(viewId/(numImgs+1),f,sprintf('Current image ID: %d',viewId));
        currImg = undistortImage(rgb2gray(imgs{viewId-1}),cameraParams);
         
        [currPoints, validIdx] = step(tracker, currImg);
        
        currPointsSURF = detectSURFFeatures(currImg,'MetricThreshold',3000,'NumOctaves',6,'NumScaleLevels',12);
        realCurrPoints = [SURFPoints(currPoints);currPointsSURF];
        
        release(tracker);
        initialize(tracker,realCurrPoints.Location,undistortImage(rgb2gray(imgs{viewId-1}),cameraParams));
        
        [worldPointsSURF, imagePointsSURF] = helperFind3Dto2DCorrespondences(vSet,cameraParams,...
            [find(validIdx),find(validIdx)],SURFPoints(currPoints));

        [orient, loc] = estimateWorldCameraPose(imagePointsSURF, worldPointsSURF,...
            cameraParams,'MaxNumTrials',50000,'Confidence',99.99,...
            'MaxReprojectionError',0.7);
        
        vSet = addView(vSet,viewId,'Points',realCurrPoints,'Orientation',orient,...
            'Location',loc);
 
        vSet = addConnection(vSet,viewId-1,viewId,'Matches',[find(validIdx),find(validIdx)]);

        if mod(viewId,2) == 1
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
    end
    
    delete(f);
end