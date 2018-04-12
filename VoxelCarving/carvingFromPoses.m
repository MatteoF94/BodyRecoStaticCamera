function voxels = carvingFromPoses(voxels,vSetL,vSetR,silhsL,silhsR,cameraParams)
    voxelMaxValue = vSetL.NumViews + vSetR.NumViews; 
    voxels(:,4) = voxelMaxValue;
    
    voxels = singleCarve(voxels,vSetL,silhsL,cameraParams,voxelMaxValue);
    voxels = singleCarve(voxels,vSetR,silhsR,cameraParams,voxelMaxValue);
    
    pcshow(voxels(voxels(:,4)>=voxelMaxValue,1:3));
end

function voxels = singleCarve(voxels,vSet,silhs,cameraParams,voxelMaxValue)
    numPoses = vSet.NumViews;
     for i=1:numPoses
         Pose = poses(vSet, i);
         Orientation = Pose.Orientation{1};
         Location    = Pose.Location{1};
         [R,t]=cameraPoseToExtrinsics(Orientation,Location);
         z=silhs{i};
         voxels = voxelCarving(z,t,R,cameraParams.IntrinsicMatrix,voxels,voxelMaxValue,10); 
    end
end