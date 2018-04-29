function scale = computeScaleFactor(imgP,imgC,inlierP,inlierC,camP,camC,knownDistance)
%
%  Input(s): 
%           imgP - first image
%           imgC - second image
%           inlierP - matched points between imgP and imgC, belonging to
%                     the first image
%           inlierC - matched points between imgP and imgC, belonging to
%                     the second image
%           camP - camera matrix related to the first image
%           camC - camera matrix related to the second image
%           knownDistance - number describing a known distance in the scene
%  Output(s): 
%           scale - scale factor between the images and the real scene
%

    % The user first selects a corresponding pair of points in both images
    figure, imshow(imgP), hold on, plot(SURFPoints(inlierP.Location));
    [x1,y1] = getpts();
    close all;
    figure, imshow(imgC), hold on, plot(SURFPoints(inlierC.Location));
    [x2,y2] = getpts();
    close all;
    
    % The points are searched among all the matched points: the correct
    % ones are such that their distance w.r.t. the points selected by the
    % user is minimal
    distances1 = zeros(2,length(inlierP));
    distances2 = zeros(2,length(inlierC));
    
    for i = 1:length(inlierP)
        ip1 = double(inlierP(i,:).Location);
        ip2 = double(inlierC(i,:).Location);
        distances1(1,i) = sqrt((x1(1)-ip1(1))^2+((y1(1)-ip1(2))^2));
        distances1(2,i) = sqrt((x1(2)-ip1(1))^2+((y1(2)-ip1(2))^2));
        distances2(1,i) = sqrt((x2(1)-ip2(1))^2+((y2(1)-ip2(2))^2));
        distances2(2,i) = sqrt((x2(2)-ip2(1))^2+((y2(2)-ip2(2))^2));
    end

    ptOld1 = inlierP(distances1(1,:)==min(distances1(1,:))).Location;
    ptOld2 = inlierP(distances1(2,:)==min(distances1(2,:))).Location;
    ptNew1 = inlierC(distances2(1,:)==min(distances2(1,:))).Location;
    ptNew2 = inlierC(distances2(2,:)==min(distances2(2,:))).Location;

    % Lastly, we triangulate the points using the camera matrices and
    % compute the scale factor between scene (described by the known
    % distance) and the images
    knownObj = triangulate([ptOld1', ptOld2'],[ptNew1', ptNew2'],camP,camC);
    scale = double(knownDistance/norm(knownObj(1,:)-knownObj(2,:)));
end