function voxels = carvingFromPoses(voxels,vSetL,vSetR,silhsL,silhsR,lLimit,rLimit,cameraParams)
%
%  Input(s):
%           voxels - set of voxels describing the whole volume around the
%                    target (raw discretisation)
%           vSetL - structure containing half camera poses ans related
%                   information
%           vSetR - structure containing the other half camera poses ans related
%                   information
%           silhsL - silhouettes associated with the camera poses in vSetL
%           silhsR - silhouettes associated with the camera poses in vSetR
%           lLimit - number of poses in vSetL to not consider, from the end
%           rLimit - number of poses in vSetR to not consider, from the end
%           cameraParams - parameters of the camera obtained through
%                          calibration
%  Output(s):
%           voxels - set of voxels describing the volume occupied by the
%                    target (refined discretisation)
%

    % Initialize the not normalised fuzzy value of each voxel
    voxelMaxValue = vSetL.NumViews + vSetR.NumViews; 
    voxels(:,4) = voxelMaxValue; 
    
    % Do voxel carving for the left poses and right poses separately
    voxels = singleCarve(voxels,vSetL,silhsL,cameraParams,voxelMaxValue,lLimit);
    voxels = singleCarve(voxels,vSetR,silhsR,cameraParams,voxelMaxValue,rLimit);
    
    pcshow(voxels(voxels(:,4)>=voxelMaxValue,1:3));
end

function voxels = singleCarve(voxels,vSet,silhs,cameraParams,voxelMaxValue,limit)
%
%  Input(s):
%           voxels - set of voxels (not or partially refined)
%           vSet - structure containing half camera poses ans related
%                  information
%           vSetR - structure containing the other half camera poses ans related
%                   information
%           cameraParams - parameters of the camera obtained through
%                          calibration
%           voxelMaxValue - maximum not normalised fuzzy value of the
%                           voxels
%           limit - number of poses to not consider in the carving, from
%                   the end of the vSet
%  Output(s):
%           voxels - set of voxels describing the volume occupied by the
%                    target (partially or completely refined discretisation)
%

    numPoses = vSet.NumViews;
     for i=1:numPoses-limit
         
         % Extract from the set the correct orientation and pose of the
         % curren camera
         Pose = poses(vSet, i);
         Orientation = Pose.Orientation{1};
         Location    = Pose.Location{1};
         [R,t]=cameraPoseToExtrinsics(Orientation,Location);
         z=silhs{i};
         
         % Do voxel carving for a single silhouette/camera pose
         voxels = voxelCarving(z,t,R,cameraParams.IntrinsicMatrix,voxels,voxelMaxValue,5); 
    end
end