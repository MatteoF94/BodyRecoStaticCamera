function showCameraPoses(vSet)
    numViews = vSet.NumViews;
    for i=1:numViews
        Pose = poses(vSet,i);
        Orientation = Pose.Orientation{1};
        Location = Pose.Location{1};
        [R,t] = cameraPoseToExtrinsics(Orientation,Location);
        plotCamera('Orientation',R,'Location',t,'Size',0.01);
        hold on;
    end
end