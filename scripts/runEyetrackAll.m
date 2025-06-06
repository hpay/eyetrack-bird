function runEyetrackAll(filepath_eye_root,downsample_eye, ploton)
% Loop eye tracking over all of the files starting with given string 
% filepath_eye_root - find all folders with this root
% downsample_eye - downsampling between QTM -> eye cameras (usually 5 or 6) - must be the
% same for all files to be combined
% ploton - make plots
% x axis = mean of p_eyes to beak. y axis pointing left, z axis up

%% May need to change these
camfilename = 'cam%i*.avi'; % Filename template for eye cameras
calib_root = 'Z:\Hannah\eyetrack\calibration'; % Root dir for camera calibrations

data_root = fileparts(filepath_eye_root);
a = dir(filepath_eye_root);
folders = {a.name};
Q_all = [];
Q_all.N_head = 0;
Q_all.p_rigidbody  = [];
Q_all.R_rigidbody = [];
E_all = [];
E_all.pupil1 = [];
E_all.cr1 = [];
E_all.pupil2 = [];
E_all.cr2 = [];
E_all.resid1 = [];
E_all.resid2 = [];
E_all.points_fraction1 = [];
E_all.points_fraction2 = [];


p_pupil_all= [];
p_cornea_all = [];
p_beak_all =[];
dist_pc_all = [];

for ii  = 1:length(folders)
    close all
    filepath_eye = fullfile(data_root, folders{ii});
    disp(filepath_eye)
    
   
    %% Get camera calibration -- folder must be named same date as recording, e.g. 220831
    temp = strfind(filepath_eye,'_');
    folder_camera_calib = filepath_eye(temp(end)+1:temp(end)+6);
    filepath_camera = fullfile(calib_root, folder_camera_calib);
    disp(filepath_camera)
    [C, A] = calibrateCameras(filepath_camera);
    if C.p_light1(1) > C.p_light2(1)
        warning('Check that light 1 and 2 are not mislabelled in lights folder')
    end
    if C.p_cam1(1) > C.p_cam2(1)
        warning('Check that cameras 1 and 2 are not mislabelled in calibration folder')
    end
    
    %% Load Qualysis rigid body
    filename_qtm = 'qtm.mat'; % 6DOF
    Q = getfield(load(fullfile(filepath_eye,filename_qtm)),'qtm');
    N_head = Q.Frames;
    
    %% Check file lengths
    cam1 = dir(fullfile(filepath_eye, sprintf(camfilename, 1)));
    cam2 = dir(fullfile(filepath_eye, sprintf(camfilename, 2)));
    vid1 = VideoReader(fullfile(cam1.folder,cam1.name)); %#ok
    vid2 = VideoReader(fullfile(cam2.folder,cam2.name)); %#ok
    N_eye1 = round(vid1.Duration*vid1.FrameRate);
    N_eye2 = round(vid2.Duration*vid2.FrameRate);
    fprintf('Video frames cam1 %i, cam2 %i\nPredicted QTM frames %i, actual QTM frames %i\n', N_eye1, N_eye2, N_eye1*downsample_eye,N_head)
    
    %% Run eye tracking (will skip if already run)
    resume_beak = 0; % Change to 1 to add more beak points
    run_beak = 1;
    if ii>1
        run_beak = 0;
    end
    resume_eye = 1;
    [E, p_pupil_cam, p_cornea_cam, p_beak_cam] = processEyetrack(filepath_eye, camfilename,  C, resume_beak, run_beak, resume_eye);
    [p_pupil_cam, p_cornea_cam, dist_pc] = correctLengthP_CR(p_pupil_cam, p_cornea_cam);
    N_eye = length(E.resid1);
    
    E_all.pupil1 = [E_all.pupil1; E.pupil1];
    E_all.cr1 = cat(1,E_all.cr1, E.cr1);
    E_all.pupil2 = [E_all.pupil2; E.pupil2];
    E_all.cr2 = cat(1, E_all.cr2, E.cr2);
    E_all.resid1 = [E_all.resid1; E.resid1];
    E_all.resid2 = [E_all.resid2; E.resid2];
    try
        E_all.points_fraction1 = [E_all.points_fraction1; E.points_fraction1];
        E_all.points_fraction2 = [E_all.points_fraction2; E.points_fraction2];
    catch % Early recordings didn't calculate this
        E_all.points_fraction1 = [E_all.points_fraction1; NaN(N_eye,1)];
        E_all.points_fraction2 = [E_all.points_fraction2; NaN(N_eye,1)];
    end
    
    % Crop QTM if longer than eye cameras
    if N_head  > N_eye*downsample_eye
        warning('Cropping QTM')
        Q.RigidBodies.Positions = Q.RigidBodies.Positions(1,:,1:N_eye*downsample_eye);
        Q.RigidBodies.Rotations = Q.RigidBodies.Rotations(1,:,1:N_eye*downsample_eye);
        N_head = N_eye*downsample_eye;
    end
    
    Q_raw.fps_head = Q.FrameRate;
    Q_raw.N_head =  N_head;
    Q_raw.p_rigidbody = squeeze(Q.RigidBodies.Positions(1,:,:))';
    Q_raw.R_rigidbody = reshape(Q.RigidBodies.Rotations(1,:,:), [3,3,N_head]); % [X Y Z]
        
    %% Convert to common reference frame
    Q_common = Q_raw;
    [p_pupil_common, p_cornea_common, p_beak_common, Q_common.p_rigidbody, Q_common.R_rigidbody]...
        = alignCommonReferenceFrame(A, p_pupil_cam, p_cornea_cam, p_beak_cam, Q_raw.p_rigidbody, Q_raw.R_rigidbody);

    
    %% Combine files
    
    p_pupil_all = [p_pupil_all; p_pupil_common];
    p_cornea_all = [p_cornea_all; p_cornea_common];
    p_beak_all = [p_beak_all; p_beak_common];
    dist_pc_all = [dist_pc_all; dist_pc];

    Q_all.fps_head = Q.FrameRate;
    Q_all.N_head = Q_all.N_head + Q_common.N_head;
    Q_all.p_rigidbody = [Q_all.p_rigidbody; Q_common.p_rigidbody];
    Q_all.R_rigidbody = cat(3, Q_all.R_rigidbody, Q_common.R_rigidbody); % [X Y Z]
    
    
end


%% Plot the distribution of eye angles relative to common reference frame (which is roughly aligned between the two cameras)

% Get a unit vector pointing in the direction of the eye
v_eye = (p_pupil_all-p_cornea_all)./sqrt(sum((p_pupil_all-p_cornea_all).^2, 2));

% Get the horizontal angle (relative to the world/motor)
yaw_eye = -real(squeeze(atan2(v_eye(:,2), v_eye(:,1))));
pitch_eye = atan2(v_eye(:,3), sqrt(v_eye(:,1).^2 + v_eye(:,2).^2));
yawd_eye = rad2deg(yaw_eye);
pitchd_eye = rad2deg(pitch_eye);
figure; plot(v_eye,'.')
figure; plot([yawd_eye pitchd_eye],'.-');
figure; histogram(yawd_eye-90, -50:1:50)
figure; histogram(yawd_eye, -180:1:180)


%% Analyze results for all session for this bird
a = strfind(filepath_eye_root,'_');
filepath_eye = [filepath_eye_root(1:a(end)-1) '_all']



[H, Ht, stats, Eout] = analyzeEyetrack(E_all, p_pupil_all, p_cornea_all, p_beak_all, Q_all,  downsample_eye, dist_pc_all);
drawnow;
H.folder_eye_calib = folders;
H.folder_camera_calib = '';
Ht.folder = {filepath_eye};
disp(Ht)
disp(stats)


%% Save results
mkdir(filepath_eye)
save(fullfile(filepath_eye, 'head_calibration.mat'),'-struct','H');
save(fullfile(filepath_eye, 'head_calibration_template.mat'),'Ht');
save(fullfile(filepath_eye, 'head_calibration_stats.mat'),'stats');
save(fullfile(filepath_eye, 'head_calibration_data.mat'),'-struct','Eout');
save(fullfile(filepath_eye, 'head_calibration_qualisys.mat'),'-struct','Q_all');

