function [xlim,ylim,zlim] = findModelBoundaries(vSetL,vSetR,silhsL,silhsR,nL,nR,cameraParams)
    [xminL,xmaxL,zminL,zmaxL] = findCamerasBoundaries(vSetL);
    [xminR,xmaxR,zminR,zmaxR] = findCamerasBoundaries(vSetR);
    
    xmin = min(xminL,xminR);
    xmax = max(xmaxL,xmaxR);
    deltaX = xmax - xmin;
    
    zmin = min(zminL,zminR);
    zmax = max(zmaxL,zmaxR);
    deltaZ = zmax - zmin;
    if(deltaZ < 3/4*deltaX)
        deltaZ = deltaX;
        zmax = zmin + deltaZ;
    end
    if(deltaX < 3/4*deltaZ)
        deltaX = deltaZ;
        xmax = deltaX/2;
        xmin = -deltaX/2;
    end
    
    deltaY = 2.5*max(deltaZ,deltaX);
    ymin = - deltaY/2;
    ymax = deltaY/2;
    
    volume = deltaZ*deltaX*deltaY;
    delta = nthroot(volume/10000000,3);
    voxelSize = [delta delta delta];
    [voxels,~,~,~,~] = initializeVoxels([xmin,xmax],[ymin,ymax],[zmin,zmax],voxelSize);
    
    voxels = carvingFromPoses(voxels,vSetL,vSetR,silhsL,silhsR,nL,nR,cameraParams);
    xlim = extendLimits([min(voxels(:,1)) max(voxels(:,1))],0.1);
    ylim = extendLimits([min(voxels(:,2)) max(voxels(:,2))],0.1);
    zlim = extendLimits([min(voxels(:,3)) max(voxels(:,3))],0.1);
end

function [xmin,xmax,zmin,zmax] = findCamerasBoundaries(vSet)
    numViews = vSet.NumViews;
    positions = zeros(3,numViews);
    
    for i = 1:numViews
        Pose = poses(vSet,i);
        Orientation = Pose.Orientation{1};
        Location = Pose.Location{1};
        [~,t] = cameraPoseToExtrinsics(Orientation,Location);
        positions(:,i) = t;
    end
    
    xmin = min(positions(1,:));
    xmax = max(positions(1,:));
    zmin = min(positions(3,:));
    zmax = max(positions(3,:));
end

function lim = extendLimits(lim,qt)
    dL = (lim(2)-lim(1))*qt;
    lim(1) = lim(1) - dL;
    lim(2) = lim(2) + dL;
end