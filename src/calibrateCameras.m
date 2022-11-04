% Calibrate dual camera system from a calibration session
% Three steps:
% 1. Standard stereo camera calibration using checkerboards
% 2. Find position of IR lights in the world
% 3. Define common reference frame using simultaneous QTM measurement
%
% Hannah Payne, Aronov Lab 2022

function [C,A] = calibrateCameras(filepath_camera_calib)

% Ask for directory if not provided
if ~exist('filepath_camera_calib','var')
    filepath_camera_calib = uigetdir('Z:\Hannah\dualcamera\calibration','Select calibration folder');
    if ~filepath_camera_calib; return; end
end

if ~exist(fullfile(filepath_camera_calib,'calib.mat'),'file')
    
    
    % Get standard stereo calibration
    square_size = 4; % (mm)
    folder1 = fullfile(filepath_camera_calib, 'cam1');
    folder2 = fullfile(filepath_camera_calib, 'cam2');
    stereo_params = myStereoCameraCalibrator(folder1,folder2,square_size);
    
    % Get the location of the IR lights
    C = calibrateLights(filepath_camera_calib, stereo_params);
    save(fullfile(filepath_camera_calib,'calib.mat'),'C')
else
    C = getfield(load(fullfile(filepath_camera_calib,'calib.mat')),'C');
    
end

if ~exist(fullfile(filepath_camera_calib, 'axes.mat'),'file')
    
    %% Detect the common axes in the dual camera and QTM reference frames
    % Provide folder containing axes.mat file from QTM and pair of images
    % named 'cam1.bmp' and 'cam2.bmp' of an three-marker triangle
    % (long edge = x axis, short edge = y axis, rotated 45 deg from x axis)
    filepath_axes = fullfile(filepath_camera_calib, 'axes');
    
    % Take image where the targets are saturated
    thresh = 250;
    try
        A  = calibrateAxesBW(filepath_axes, C.stereo_params, thresh);
    catch
        A  = calibrateAxesOld(filepath_axes, C.stereo_params, thresh);
    end
    %% Save results
    save(fullfile(filepath_camera_calib,'axes.mat'),'A');
    
else
    
    A = getfield(load(fullfile(filepath_camera_calib,'axes.mat')), 'A');
    
    
end
