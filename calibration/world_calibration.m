%% calibration routine
addpath('./matlab_support');

load './data/scene.mat'
final_poses = final_poses(1:19);
%% configuration
improve = true;% use proposed alogrithm to improve transformations
squareSize = 0.14;  % in units of 'm'

imageBaseName = './data/calibrate_';% path and start of filename to images
imageExtension = '.png';


% one camera has images $(imageBaseName)$(i)a$(imageExtension)
% the other has $(imageBaseName)$(i)b$(imageExtension)
% where $(i) is the pair number


%  The stereoCameraCalibrator app can also be used to get a visual rep of
%  the initial calibration

%% read end-effector poses
% the poses used are the ones generated from the calibration config
% poses

%% detect calibration pattern
n_imagePairs = length(final_poses);

images1 = cell(1, n_imagePairs);
images2 = images1;

for i=1:n_imagePairs
    images1{i} = [imageBaseName num2str(i) 'a' imageExtension];
    images2{i} = [imageBaseName num2str(i) 'b' imageExtension];
end

% detect
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(images1, images2);
final_poses(~imagesUsed) = []; 
% make a pattern from the data
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

%% calibrate
I1 = imread(images1{1});
[mrows, ncols, ~] = size(I1);

% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'm', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

%% calculate camera to world transforms

[ T_cw1, T_cw2, T_eo, stereoParams ] = findTransformations( stereoParams, final_poses );

%% optionally improve by proposed algorithm  
if improve
    [stereoParams, ~, T_cw1] = estimateHH( stereoParams, imagePoints, T_eo, final_poses, T_cw1, false );
    T = [ stereoParams.RotationOfCamera2' stereoParams.TranslationOfCamera2'; 0 0 0 1];
    T_cw2 = T*T_cw1;
end

%% make cameras
camera1 = stereoParams.CameraParameters1.IntrinsicMatrix'*[ eye(3) [0 0 0]']/T_cw1;
camera2 = stereoParams.CameraParameters2.IntrinsicMatrix'*[ eye(3) [0 0 0]']/T_cw2;


%% save results
fid = fopen('../data/camera_parameters.txt', 'wt');
if fid<0
    error('Could not open camera parameters\n');
end

% file layout:
% Serial number of camera 1
% camera 1 - 1x12, each row after another
% Serial number of camera 2
% camera 2 - 1x12, each row after another

% read serial numbers
fid_serials = fopen('./data/cameraIDs.txt', 'rt');
if fid_serials<0
    error('Could not open camera serial ids');
end
ID1 = fgetl(fid_serials);
ID2 = fgetl(fid_serials);
fclose(fid_serials);

% camera 1
fprintf(fid, '%s\n', ID1 );

for i=1:3
    fprintf(fid, '%f %f %f %f',camera1(i,:) );
end
fprintf(fid, '\n');

% camera 2
fprintf(fid, '%s\n', ID2 );

for i=1:3
    fprintf(fid, '%f %f %f %f',camera2(i,:) );
end

fclose(fid);
%% visualize
% Visualize pattern locations
h2=figure; showExtrinsics(stereoParams, 'CameraCentric');