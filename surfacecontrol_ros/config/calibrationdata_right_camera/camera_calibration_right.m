% Auto-generated by cameraCalibrator app on 10-Nov-2022
%-------------------------------------------------------


% Define images to process
right_imageFileNames = {'/home/terabotics/SurfaceControl_ws/src/surfacecontrol_ros/config/calibrationdata_right_camera/left-0000.png',...
    '/home/terabotics/SurfaceControl_ws/src/surfacecontrol_ros/config/calibrationdata_right_camera/left-0001.png',...
    '/home/terabotics/SurfaceControl_ws/src/surfacecontrol_ros/config/calibrationdata_right_camera/left-0005.png',...
    '/home/terabotics/SurfaceControl_ws/src/surfacecontrol_ros/config/calibrationdata_right_camera/left-0008.png',...
    '/home/terabotics/SurfaceControl_ws/src/surfacecontrol_ros/config/calibrationdata_right_camera/left-0011.png',...
    '/home/terabotics/SurfaceControl_ws/src/surfacecontrol_ros/config/calibrationdata_right_camera/left-0016.png',...
    '/home/terabotics/SurfaceControl_ws/src/surfacecontrol_ros/config/calibrationdata_right_camera/left-0018.png',...
    '/home/terabotics/SurfaceControl_ws/src/surfacecontrol_ros/config/calibrationdata_right_camera/left-0030.png',...
    '/home/terabotics/SurfaceControl_ws/src/surfacecontrol_ros/config/calibrationdata_right_camera/left-0031.png',...
    '/home/terabotics/SurfaceControl_ws/src/surfacecontrol_ros/config/calibrationdata_right_camera/left-0032.png',...
    };
%%
% Detect calibration pattern in images
detector_right = vision.calibration.monocular.CheckerboardDetector();
[imagePoints_right, imagesUsed_right] = detectPatternPoints(detector_right, right_imageFileNames);
right_imageFileNames = right_imageFileNames(imagesUsed_right);

% Read the first image to obtain image size
originalImage_right = imread(right_imageFileNames{1});
[mrows_right, ncols_right, ~] = size(originalImage_right);

% Generate world coordinates for the planar pattern keypoints
squareSize = 10;  % in units of 'millimeters'
worldPoints_right = generateWorldPoints(detector_right, 'SquareSize', squareSize);

% Calibrate the camera
[cameraParams_right, imagesUsed_right, estimationErrors_right] = estimateCameraParameters(imagePoints_right, worldPoints_right, ...
    'EstimateSkew', true, 'EstimateTangentialDistortion', true, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows_right, ncols_right]);

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams_right);

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams_right, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors_right, cameraParams_right);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage_right = undistortImage(originalImage_right, cameraParams_right);
%%
msg_right = rosmessage("sensor_msgs/CameraInfo","DataFormat","struct");
[msgOut_right] = rosWriteCameraInfo(msg_right,toStruct(cameraParams_right));
% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')
