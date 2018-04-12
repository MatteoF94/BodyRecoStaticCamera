function voxels = singleCarve(voxels,vSet,silhs,cameraParams,voxelMaxValue)
    numPoses = vSet.NumViews;
     for i=1:numPoses
         Pose = poses(vSet, i);
         Orientation = Pose.Orientation{1};
         Location    = Pose.Location{1};
         [R,t]=cameraPoseToExtrinsics(Orientation,Location);
         z=silhs{i};
         voxels = voxelCarving(z,t,R,cameraParams.IntrinsicMatrix,voxels,voxelMaxValue,0); 
    end
end