function cameraParams = calibrationDemo3()
%
%  Input(s): 
%  Output(s): 
%           cameraParams - parameters of the camera for demo 3 obtained
%                          through calibration
%

    % Define images to process
    imageFileNames = {strcat(pwd,'/Dataset/Demo3/calib/IMG_0001.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0002.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0003.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0004.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0005.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0006.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0007.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0008.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0009.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0010.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0011.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0012.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0013.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0014.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0017.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0018.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0021.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0026.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0027.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0028.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0029.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0030.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0031.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0034.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0035.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0036.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0037.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0038.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0039.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0040.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0041.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0042.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0043.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0044.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0045.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0046.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0047.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0048.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0049.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0050.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0051.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0052.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0053.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0054.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0056.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0057.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0058.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0059.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0060.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0061.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0062.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0063.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0064.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0065.jpg'),...
        strcat(pwd,'/Dataset/Demo3/calib/IMG_0066.jpg'),...
        };

    % Detect checkerboards in images
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
    imageFileNames = imageFileNames(imagesUsed);

    % Read the first image to obtain image size
    originalImage = imread(imageFileNames{1});
    [mrows, ncols, ~] = size(originalImage);

    % Generate world coordinates of the corners of the squares
    squareSize = 30;  % in units of 'millimeters'
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

    % Calibrate the camera
    [cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
        'EstimateSkew', true, 'EstimateTangentialDistortion', false, ...
        'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
        'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
        'ImageSize', [mrows, ncols]);

    % View reprojection errors
    %h1=figure; showReprojectionErrors(cameraParams);

    % Visualize pattern locations
    %h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

    % Display parameter estimation errors
    %displayErrors(estimationErrors, cameraParams);

    % For example, you can use the calibration data to remove effects of lens distortion.
    %undistortedImage = undistortImage(originalImage, cameraParams);

    % See additional examples of how to use the calibration data.  At the prompt type:
    % showdemo('MeasuringPlanarObjectsExample')
    % showdemo('StructureFromMotionExample')
end