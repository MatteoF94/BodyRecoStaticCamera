function showCameraPoses(vSet)
%
%  Input(s):
%           vSet - structure containing information about camera poses
%

    numViews = vSet.NumViews;
    for i=1:numViews
        
        % Extract the orientation and location of each camera in the
        % structure and show them together
        Pose = poses(vSet,i);
        Orientation = Pose.Orientation{1};
        Location = Pose.Location{1};
        [R,t] = cameraPoseToExtrinsics(Orientation,Location);
        plotCamera('Orientation',R,'Location',t,'Size',0.01);
        hold on;
    end
end