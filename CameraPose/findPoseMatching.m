function vSet = findPoseMatching(imgs,cameraParams,knownDistance)
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

    % Initialize the view set, defining also fixed values
    MATCH_THRESH = 10;
    MAX_RATIO = 0.7;
    
    vSet = viewSet;

    % Extract features from the first image
    viewId = 1;
    firstImg = undistortImage(rgb2gray(imgs{viewId}),cameraParams);
    startPoints = detectSURFFeatures(firstImg,'MetricThreshold',2000,'NumOctaves',12,'NumScaleLevels',12);
    [firstFeatures,validFirstPts] = extractFeatures(firstImg,startPoints);
    vSet = addView(vSet,viewId,'Points',startPoints,'Orientation',eye(3),'Location',[0 0 0]);
    
    % Extract features from the second image
    viewId = 2;
    currImg = undistortImage(rgb2gray(imgs{viewId}),cameraParams);
    currPoints = detectSURFFeatures(currImg,'MetricThreshold',2000,'NumOctaves',12,'NumScaleLevels',12);
    [currFeatures,validCurrPts] = extractFeatures(currImg,currPoints);
    
    % Find matching features between the two images
    indexPairs = matchFeatures(firstFeatures,currFeatures,'MatchThreshold',MATCH_THRESH,...
        'MaxRatio',MAX_RATIO,'Unique',true);
    
    matchingStartPts = validFirstPts(indexPairs(:,1),:);
    matchingCurrPts = validCurrPts(indexPairs(:,2),:);
    
    % Estimate the essential matrix from the previously found matching features
    [E, inlierIdx] = estimateEssentialMatrix(matchingStartPts,matchingCurrPts,cameraParams,...
        'Confidence',99.99,'MaxDistance',5,'MaxNumTrials',500);
    inlierPointsFirst = matchingStartPts(inlierIdx); 
    inlierPointsCurr = matchingCurrPts(inlierIdx); 
    
    % Find the second camera pose, fixing the first
    [orient,loc,~] = relativeCameraPose(E,cameraParams,inlierPointsFirst,inlierPointsCurr);
    [R12,t12] = cameraPoseToExtrinsics(orient,loc);
    camMatrix2 = cameraMatrix(cameraParams, R12, t12); 
    camMatrix1 = cameraMatrix(cameraParams,eye(3),[0 0 0]); 
    
    % Adjust the camera poses computing the scale factor between images and
    % scene
    scaleFactor = computeScaleFactor(firstImg,currImg,...
                                     inlierPointsFirst,inlierPointsCurr,...
                                     camMatrix1,camMatrix2,knownDistance);
    loc = loc*scaleFactor;
    
    vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, 'Location', loc); 
    vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);
    
    % Estimate the other camera poses
    vSet = estimationCompletion(imgs,cameraParams,vSet,currFeatures,validCurrPts); 
end

function vSet = estimationCompletion(imgs,cameraParams,vSet,currFeatures,validCurrFeat)
%
%  Input(s): 
%           imgs - sequence of images for which we want to find the
%                  correspondant camera poses
%           cameraParams - the parameters describing the cameras obtained
%                          through previous calibration
%           vSet - the structured set containing the poses found so far
%           currFeatures - features obtained from the second image
%           validCurrFeat - valid points, related to the features of the
%                           second image
%  Output(s): 
%           vSet - complete structured set containing all the camera poses
%

    numImgs = length(imgs);
    prevFeatures = currFeatures;
    validPrevFeat = validCurrFeat;
    
    for viewId = 3:numImgs
        
        % Extract features from the viewId-th images
        currImg = undistortImage(rgb2gray(imgs{viewId}),cameraParams);
        currPoints = detectSURFFeatures(currImg,'MetricThreshold',500,'NumOctaves',18,'NumScaleLevels',18);
        [currFeatures,validCurrFeat] = extractFeatures(currImg,currPoints);
        indexPairs = matchFeatures(prevFeatures,currFeatures,'MatchThreshold',10,...
        'MaxRatio',0.7,'Unique',true);
    
        % Find matching features between the current and previous image
        prevImg = undistortImage(rgb2gray(imgs{viewId}),cameraParams);
        matchingPtsPrev = validPrevFeat(indexPairs(:,1),:);
        matchingPtsCurr = validCurrFeat(indexPairs(:,2),:);
        showMatchedFeatures(prevImg,currImg,matchingPtsPrev,matchingPtsCurr);

        [worldPointsSURF, imagePointsSURF] = helperFind3Dto2DCorrespondences(vSet,cameraParams,...
            indexPairs,currPoints);

        % Estimate the current camera pose
        [orient, loc] = estimateWorldCameraPose(imagePointsSURF, worldPointsSURF,...
            cameraParams,'MaxNumTrials',50000,'Confidence',99.99,...
            'MaxReprojectionError',0.7);

        vSet = addView(vSet,viewId,'Points',currPoints,'Orientation',orient,...
            'Location',loc);
        vSet = addConnection(vSet,viewId-1,viewId,'Matches',indexPairs);

        % At every even image, perform bundle adjustment on a small window
        % of images (to avoid high computational costs)
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
        
        % Save features and valid points for the next iteration
        prevFeatures = currFeatures;
        validPrevFeat = validCurrFeat;
    end
end