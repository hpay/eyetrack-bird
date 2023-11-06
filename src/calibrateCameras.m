% Calibrate dual camera system from a calibration session
% Three steps:
% 1. Standard stereo camera calibration using checkerboards
% 2. Find position of IR lights in the world
% 3. Define common reference frame using simultaneous QTM measurement
%
% Returns: 
% C: camera calibrations
% A: co-registration of qtm and dual camera system axes
% A.R_eye: rotation matrix for axes as seen by dual cameras
% A.R_head: rotation matrix for axes as seen by QTM
% A.R_45: an intermediate step
% A.p0_eye: origin location for axes as seen by dual cameras
% A.p0_head: origin for axes as seen by QTM
%
% All QTM measurements ("head") are relative to the QTM reference frame (defined by the
% separate calibration step conducted in QTM using the triangle and wand)
%
% All dual camera ("eye") measurements are relative to the dual camera
% reference frame (defined by the camera calibrations C)
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
