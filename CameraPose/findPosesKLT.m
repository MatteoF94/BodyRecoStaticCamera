function vSet = findPosesKLT(imgs,cameraParams,knownDistance)
%
%  Input(s): 
%           imgs - sequence of images for which we want to find the
%                  correspondant camera poses
%           cameraParams - the parameters describing the cameras obtained
%                          through previous calibration
%           knownDistance - number describing a known distance in the scene
%  Output(s): 
%           vSet - structured set containing camera poses, their
%                  correspondant images and the connections between images/poses
%

    % Initialize the view set and the point tracker
    vSet = viewSet;
    tracker = vision.PointTracker('MaxBidirectionalError',0.5, 'NumPyramidLevels',10);

    % Extract features from the first image
    viewId = 1;
    prevImg = undistortImage(rgb2gray(imgs{viewId}),cameraParams);
    prevPoints = detectSURFFeatures(prevImg,'MetricThreshold',1000,'NumOctaves',12,'NumScaleLevels',12);
    prevPoints = prevPoints.Location;
    initialize(tracker,prevPoints,prevImg);
    vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', eye(3),...
        'Location',[0 0 0]);

    % Track the points in the second image and extract its features
    viewId = 2;
    currImg = undistortImage(rgb2gray(imgs{viewId}),cameraParams);
    currPointsSURF = detectSURFFeatures(currImg,'MetricThreshold',1000,'NumOctaves',12,'NumScaleLevels',12);
    [currPoints, validIdx] = step(tracker, currImg);
    
    matchingPtsPrev = prevPoints(validIdx,:);
    matchingPtsCurr = currPoints(validIdx,:);

    % Estimate the essential matrix from the previously found matching features
    [E, inlierIdx] = estimateEssentialMatrix(matchingPtsPrev,matchingPtsCurr,cameraParams,...
        'Confidence',99.99,'MaxDistance',5,'MaxNumTrials',3000);
    inlierPointsPrev = matchingPtsPrev(inlierIdx,:); 
    inlierPointsCurr = matchingPtsCurr(inlierIdx,:); 

    % Find the second camera pose, fixing the first
    [orient,loc,~] = relativeCameraPose(E,cameraParams,inlierPointsPrev,inlierPointsCurr);
    [R12,t12] = cameraPoseToExtrinsics(orient,loc);
    camMatrix2 = cameraMatrix(cameraParams, R12, t12); 
    camMatrix1 = cameraMatrix(cameraParams,eye(3),[0 0 0]); 
    
    % Adjust the camera poses computing the scale factor between images and
    % scene
    scaleFactor = computeScaleFactor(prevImg,currImg,...
                                     SURFPoints(inlierPointsPrev),SURFPoints(inlierPointsCurr),...
                                     camMatrix1,camMatrix2,knownDistance);
    loc = loc*scaleFactor;
    
    % Group tracked points and extracted features of the second image
    realCurrPoints = [SURFPoints(currPoints);currPointsSURF];
    
    vSet = addView(vSet, viewId, 'Points', realCurrPoints, 'Orientation', orient, 'Location', loc); 
    vSet = addConnection(vSet, viewId-1, viewId, 'Matches', [find(validIdx),find(validIdx)]);
    
    % Estimate the other camera poses
    vSet = estimationCompletion(imgs,cameraParams,vSet,tracker,realCurrPoints); 
end

function vSet = estimationCompletion(imgs,cameraParams,vSet,tracker,points)
%
%  Input(s): 
%           imgs - sequence of images for which we want to find the
%                  correspondant camera poses
%           cameraParams - the parameters describing the cameras obtained
%                          through previous calibration
%           vSet - the structured set containing the poses found so far
%           tracker - is the object that performs KLT tracking
%           points - all the valid points of the second image (tracked +
%                    extracted)
%  Output(s): 
%           vSet - complete structured set containing all the camera poses
%

    numImgs = length(imgs);
    
    % Reset the tracker and initialize it anew from the second image
    release(tracker);
    initialize(tracker,points.Location,undistortImage(rgb2gray(imgs{1}),cameraParams));
    f = waitbar(2/(numImgs),'1','Name','Camera poses estimation',...
    'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');

    for viewId = 3:numImgs
        waitbar(viewId/(numImgs),f,sprintf('Current image ID: %d',viewId));
        currImg = undistortImage(rgb2gray(imgs{viewId-1}),cameraParams);
        
        % Track the previous points in the current image and extract its
        % features, grouping them together
        [currPoints, validIdx] = step(tracker, currImg);
        
        currPointsSURF = detectSURFFeatures(currImg,'MetricThreshold',500,'NumOctaves',18,'NumScaleLevels',18);
        realCurrPoints = [SURFPoints(currPoints);currPointsSURF];
        
        % Reset the tracker and initialize it anew from the current image
        release(tracker);
        initialize(tracker,realCurrPoints.Location,undistortImage(rgb2gray(imgs{viewId-1}),cameraParams));
        
        [worldPointsSURF, imagePointsSURF] = helperFind3Dto2DCorrespondences(vSet,cameraParams,...
            [find(validIdx),find(validIdx)],SURFPoints(currPoints));

        % Estimate the current camera pose
        [orient, loc] = estimateWorldCameraPose(imagePointsSURF, worldPointsSURF,...
            cameraParams,'MaxNumTrials',50000,'Confidence',99.99,...
            'MaxReprojectionError',0.7);
        
        vSet = addView(vSet,viewId,'Points',realCurrPoints,'Orientation',orient,...
            'Location',loc);
        vSet = addConnection(vSet,viewId-1,viewId,'Matches',[find(validIdx),find(validIdx)]);

        % At every even image, perform bundle adjustment on a small window
        % of images (to avoid high computational costs)
        if mod(viewId,2) == 0
            windowSize = 4;
            startFrame = max(1,viewId - windowSize);
            tracks = findTracks(vSet, startFrame:viewId);
            camPoses = poses(vSet, 1:viewId);
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