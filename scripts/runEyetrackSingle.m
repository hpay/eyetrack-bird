function runEyetrackSingle(filepath_eye, downsample_eye)

%% May need to change these 
camfilename = 'cam%i*.avi'; % Filename template for eye cameras
calib_root = 'Z:\Hannah\eyetrack\calibration'; % Root dir for camera calibrations

%% Get camera calibration -- folder must be named same date as recording, e.g. 220831
temp = strfind(filepath_eye,'_');
folder_camera_calib = filepath_eye(temp(end)+1:temp(end)+6);
filepath_camera = fullfile(calib_root, folder_camera_calib)
[C,A] = calibrateCameras(filepath_camera);
disp(filepath_eye)

%% Run eye tracking (will skip if already run)
resume_beak = 0;
run_beak = 1;
resume_eye = 1;

[E, p_pupil_cam, p_cornea_cam, p_beak_cam] = processEyetrack(filepath_eye, camfilename,  C, resume_beak, run_beak, resume_eye);
[p_pupil_cam, p_cornea_cam, dist_pc] = correctLengthP_CR(p_pupil_cam, p_cornea_cam);

%% Load Qualysis rigid body
filename_qtm = 'qtm.mat'; % 6DOF
Q = getfield(load(fullfile(filepath_eye,filename_qtm)),'qtm');
Q_raw = [];
Q_raw.N_head = Q.Frames;
Q_raw.fps_head = Q.FrameRate;
Q_raw.p_rigidbody = squeeze(Q.RigidBodies.Positions(1,:,:))';
Q_raw.R_rigidbody = reshape(Q.RigidBodies.Rotations(1,:,:), [3,3,Q.Frames]); % [X Y Z]

%% Convert to common reference frame
Q_common = Q_raw;
[p_pupil_common, p_cornea_common, p_beak_common, Q_common.p_rigidbody, Q_common.R_rigidbody]...
    = alignCommonReferenceFrame(A, p_pupil_cam, p_cornea_cam, p_beak_cam, Q_raw.p_rigidbody, Q_raw.R_rigidbody);

%% Analyze results
[H, Ht] = analyzeEyetrack(E, p_pupil_common, p_cornea_common, p_beak_common, Q_common, downsample_eye, dist_pc);
H.folder_eye_calib = filepath_eye;
H.folder_camera_calib = filepath_camera;
Ht.folder = filepath_eye;
disp(Ht)

%% Save results
save(fullfile(filepath_eye, 'head_calibration.mat'),'-struct','H');
save(fullfile(filepath_eye, 'head_calibration_template.mat'),'Ht');

