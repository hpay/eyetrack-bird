% pptDefaults
data_root = 'Z:\Hannah\eyetrack\behavior';
code_root = fileparts(fileparts(which(mfilename)));
addpath(fullfile(code_root,'src'));
ploton = 1;

%% Process a single calibration of cameras
filepath_camera = 'Z:\Hannah\eyetrack\calibration\230210';
[C,A] = calibrateCameras(filepath_camera);

%% Run a single eye tr acking file
% dir_root = 'HC08_221028b'; %IND102_220707a
% dir_root = 'HC11_230201a';
% downsample_eye = 6;
% runEyetrackSingle(fullfile(data_root, dir_root),downsample_eye, ploton)

%% Run a set of eye tracking files for the same bird
% dir_root = 'HC07_23*'; % Best example HC07?
% dir_root = 'HC10_22*'; 
% dir_root = 'HC11_*'; 
% downsample_eye = 6;

dir_root = 'HC12_*';
downsample_eye = [5 6]; % Set the downsampling rate at which the eye camera data was collected relative to QTM

runEyetrackAll(fullfile(data_root, dir_root),downsample_eye, ploton)

return

%% Run analysis for all birds (to re-run raw eye tracking, delete the results file e.g. eye.mat and eye_trangulate.mat)
dir_roots = {'IND102_22*', 'HC05_22*', 'HC06_22*','HC07_22*','HC08_22*','HC09_22*','HC10_22*','HC11_22*','HC12_22*'} 
downsample_eyes = [6 6 6 6 6 6 6 6 5];
% NOTE: 'ROS38_220308*','TRQ180_220310*','CHC37_220310*', only had two head markers, and acrylic barrier - up through 220630, possibly less accurate?
% NOTE: IND102 was measured before and after change on several days
% NOTE: RBY47_221013 is the same bird as HC06_221013 before surgery, could
% use instead if needed

for ii = 1:length(dir_roots)
    runEyetrackAll(fullfile(data_root, dir_roots{ii}), downsample_eye, ploton)   
end

%% Plot results & give stats for all birds
plotEyetrack(data_root, dir_roots)

