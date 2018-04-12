function scale = computeScaleFactor(imgP,imgC,inlierP,inlierC,camP,camC)
    figure, imshow(imgP), hold on, plot(inlierP);
    [x1,y1] = getpts();
    close all;
    figure, imshow(imgC), hold on, plot(inlierC);
    [x2,y2] = getpts();
    close all;
    
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

    knownObj = triangulate([ptOld1', ptOld2'],[ptNew1', ptNew2'],camP,camC);
    scale = double(0.4/norm(knownObj(1,:)-knownObj(2,:)));
end